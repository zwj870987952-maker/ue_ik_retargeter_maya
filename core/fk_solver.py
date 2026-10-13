# -*- coding: utf-8 -*-
"""
FK Chain solver.
Directly implements UE 5.8's FChainDecoderFK and FChainEncoderFK (FKChainsOp.cpp).
Handles forward kinematics rotation transfer, parent frame reprojection,
and chain length parameterization using Maya OpenMaya 2.0 row-vector quaternions.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import slerp, quat_between_vectors, get_world_transform


class FKChainSolver(object):
    """
    Retargets rotational poses from a source BoneChain to a target BoneChain.
    Directly ports UE 5.8's FKChainsOp:
    1. Re-projects source chain into target parent space (TransformCurrentChainTransforms).
    2. Supports Interpolated (arc-length parameterization) and OneToOne modes.
    3. Applies world rotation deltas onto the aligned target retarget pose.
    """

    def __init__(self, mapping, target_pose_mgr=None, source_pose_mgr=None):
        self.mapping = mapping
        self.src_chain = mapping.source_chain
        self.tgt_chain = mapping.target_chain
        self.target_pose_mgr = target_pose_mgr
        self.source_pose_mgr = source_pose_mgr

        self.target_aligned_rot = {}
        self.from_to_rotation_deltas = {}

        self.update_retarget_pose()

    def update_retarget_pose(self):
        """Updates cached aligned rest rotations and FromTo rotation deltas."""
        self.target_aligned_rot = {}
        self.from_to_rotation_deltas = {}

        for tgt_idx, tgt_bone in enumerate(self.tgt_chain.bones):
            if self.target_pose_mgr:
                tgt_rest_rot = self.target_pose_mgr.get_aligned_rest_rotation(tgt_bone)
                tgt_rest_pos = self.target_pose_mgr.get_aligned_rest_position(tgt_bone)
            else:
                tgt_rest_rot = self.tgt_chain.skeleton.rest_world_rot[tgt_bone]
                tgt_rest_pos = self.tgt_chain.skeleton.rest_world_pos[tgt_bone]
            self.target_aligned_rot[tgt_bone] = tgt_rest_rot

            tgt_parent = self.tgt_chain.skeleton.get_parent_joint(tgt_bone)
            if tgt_parent and self.target_pose_mgr:
                tgt_parent_pos = self.target_pose_mgr.get_aligned_rest_position(tgt_parent)
            elif tgt_parent:
                tgt_parent_pos = self.tgt_chain.skeleton.rest_world_pos[tgt_parent]
            else:
                tgt_parent_pos = tgt_rest_pos

            tgt_dir = (tgt_rest_pos - tgt_parent_pos).normal() if (tgt_rest_pos - tgt_parent_pos).length() > 1e-4 else om.MVector(0, 1, 0)

            src_idx = min(tgt_idx, len(self.src_chain.bones) - 1)
            src_bone = self.src_chain.bones[src_idx]
            src_rest_pos = self.src_chain.skeleton.rest_world_pos[src_bone]
            src_parent = self.src_chain.skeleton.get_parent_joint(src_bone)
            src_parent_pos = self.src_chain.skeleton.rest_world_pos[src_parent] if src_parent else src_rest_pos
            src_dir = (src_rest_pos - src_parent_pos).normal() if (src_rest_pos - src_parent_pos).length() > 1e-4 else om.MVector(0, 1, 0)

            ref_pose_delta = quat_between_vectors(src_dir, tgt_dir)
            self.from_to_rotation_deltas[tgt_bone] = ref_pose_delta

    def solve(self, source_current_pose_dict, target_current_pose_dict=None):
        """
        Calculates new world rotations for all target joints in this chain.
        Note on Maya OpenMaya MQuaternion (row-vector left-to-right):
        - World rotation delta: rot_delta = src_init_rot.inverse() * src_curr_rot
        - Apply world delta to target aligned rest pose: final_rot = tgt_rest_rot * rot_delta
        """
        solved_rotations = {}

        # Parent frame compensation (UE 5.8 FKChainsOp TransformCurrentChainTransforms)
        src_parent = self.src_chain.skeleton.get_parent_joint(self.src_chain.start_bone)
        tgt_parent = self.tgt_chain.skeleton.get_parent_joint(self.tgt_chain.start_bone)

        parent_world_diff = om.MQuaternion.kIdentity
        if src_parent and tgt_parent and src_parent in source_current_pose_dict:
            src_parent_init_rot = self.src_chain.skeleton.rest_world_rot[src_parent]
            if self.target_pose_mgr:
                tgt_parent_init_rot = self.target_pose_mgr.get_aligned_rest_rotation(tgt_parent)
            else:
                tgt_parent_init_rot = self.tgt_chain.skeleton.rest_world_rot[tgt_parent]

            if target_current_pose_dict and tgt_parent in target_current_pose_dict:
                _, tgt_parent_curr_rot = target_current_pose_dict[tgt_parent]
            else:
                _, tgt_parent_curr_rot = get_world_transform(tgt_parent)

            src_parent_curr_rot = source_current_pose_dict[src_parent][1]

            # World delta of source parent vs world delta of target parent
            src_parent_delta = (src_parent_init_rot.inverse() * src_parent_curr_rot).normal()
            tgt_parent_delta = (tgt_parent_init_rot.inverse() * tgt_parent_curr_rot).normal()

            # How much target parent's world delta differs from source parent's world delta
            parent_world_diff = (src_parent_delta.inverse() * tgt_parent_delta).normal()

        for tgt_idx, tgt_bone in enumerate(self.tgt_chain.bones):
            tgt_param = self.tgt_chain.params[tgt_idx]

            if self.mapping.rotation_mode == "OneToOne" or len(self.src_chain.bones) == 1:
                src_idx = min(tgt_idx, len(self.src_chain.bones) - 1)
                src_bone = self.src_chain.bones[src_idx]
                _, src_curr_rot = source_current_pose_dict[src_bone]
                src_init_rot = self.src_chain.skeleton.rest_world_rot[src_bone]
            else:
                # Interpolated mode (UE 5.8 Default)
                _, src_curr_rot = self.src_chain.get_interpolated_transform_at_param(
                    tgt_param, source_current_pose_dict, rest_pose=False
                )
                _, src_init_rot = self.src_chain.get_interpolated_transform_at_param(
                    tgt_param, source_current_pose_dict, rest_pose=True, pose_mgr=self.source_pose_mgr
                )

            # World rotation delta of source bone from rest pose to current animated pose
            rot_delta = (src_init_rot.inverse() * src_curr_rot).normal()

            # Compensate for any parent rotation difference (e.g. when parent chain alpha < 1.0)
            compensated_delta = (rot_delta * parent_world_diff).normal()

            # Apply world delta to target aligned rest rotation
            tgt_rest_rot = self.target_aligned_rot.get(tgt_bone, self.tgt_chain.skeleton.rest_world_rot[tgt_bone])
            final_rot = (tgt_rest_rot * compensated_delta).normal()

            if self.mapping.rotation_alpha < 1.0 - 1e-4:
                final_rot = slerp(tgt_rest_rot, final_rot, self.mapping.rotation_alpha).normal()

            solved_rotations[tgt_bone] = final_rot

        return solved_rotations
