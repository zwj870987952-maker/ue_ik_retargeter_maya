# -*- coding: utf-8 -*-
"""
FK Chain solver.
Directly implements UE5's FChainDecoderFK and FChainEncoderFK (FKChainsOp.cpp).
Handles forward kinematics rotation transfer, parent frame reprojection,
and chain length parameterization without rotational skewing.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import slerp, quat_between_vectors, get_world_transform


class FKChainSolver(object):
    """
    Retargets rotational poses from a source BoneChain to a target BoneChain.
    Directly ports UE5's FKChainsOp:
    1. Re-projects source chain into target parent space (TransformCurrentChainTransforms).
    2. Supports Interpolated (arc-length parameterization) and OneToOne modes.
    3. Aligns joint orientations across differing rest poses via FromToGlobalRotationDeltas.
    """

    def __init__(self, mapping, target_pose_mgr=None):
        self.mapping = mapping
        self.src_chain = mapping.source_chain
        self.tgt_chain = mapping.target_chain
        self.target_pose_mgr = target_pose_mgr

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

            # Calculate direction vectors in rest pose (UE5 FromToGlobalRotationDeltas)
            tgt_parent = self.tgt_chain.skeleton.get_parent_joint(tgt_bone)
            if tgt_parent and self.target_pose_mgr:
                tgt_parent_pos = self.target_pose_mgr.get_aligned_rest_position(tgt_parent)
            elif tgt_parent:
                tgt_parent_pos = self.tgt_chain.skeleton.rest_world_pos[tgt_parent]
            else:
                tgt_parent_pos = tgt_rest_pos

            tgt_dir = (tgt_rest_pos - tgt_parent_pos).normal() if (tgt_rest_pos - tgt_parent_pos).length() > 1e-4 else om.MVector(0, 1, 0)

            # Corresponding source bone
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
        Follows UE5 FKChainsOp::DecodePose with parent-relative compensation.
        """
        solved_rotations = {}

        # 1. Transform source chain root to match target parent current orientation
        # (UE5 FKChainsOp.cpp lines 167-171: TransformCurrentChainTransforms)
        src_parent = self.src_chain.skeleton.get_parent_joint(self.src_chain.start_bone)
        tgt_parent = self.tgt_chain.skeleton.get_parent_joint(self.tgt_chain.start_bone)

        parent_offset_rot = om.MQuaternion.kIdentity
        if src_parent and tgt_parent:
            # Source chain parent initial global rotation
            src_parent_init_rot = self.src_chain.skeleton.rest_world_rot[src_parent]
            # Target chain parent initial global rotation
            if self.target_pose_mgr:
                tgt_parent_init_rot = self.target_pose_mgr.get_aligned_rest_rotation(tgt_parent)
            else:
                tgt_parent_init_rot = self.tgt_chain.skeleton.rest_world_rot[tgt_parent]

            # Current target parent rotation
            if target_current_pose_dict and tgt_parent in target_current_pose_dict:
                _, tgt_parent_curr_rot = target_current_pose_dict[tgt_parent]
            else:
                _, tgt_parent_curr_rot = get_world_transform(tgt_parent)

            # Delta between source parent and target parent in rest pose
            # SourceChainParentInitialDelta = SourceParentInit * TargetParentInit^-1
            src_tgt_parent_init_delta = (src_parent_init_rot * tgt_parent_init_rot.inverse()).normal()

            # Source parent current animated rotation
            src_parent_curr_rot = source_current_pose_dict[src_parent][1]

            # Transformed target parent frame
            # Target's current parent transformed into source coordinate frame
            src_parent_aligned_rot = (src_tgt_parent_init_delta * tgt_parent_curr_rot).normal()

            # Compensation offset: how much source parent rotated relative to target's current parent
            parent_offset_rot = (src_parent_aligned_rot * src_parent_curr_rot.inverse()).normal()

        for tgt_idx, tgt_bone in enumerate(self.tgt_chain.bones):
            tgt_param = self.tgt_chain.params[tgt_idx]

            if self.mapping.rotation_mode == "OneToOne":
                src_idx = min(tgt_idx, len(self.src_chain.bones) - 1)
                src_bone = self.src_chain.bones[src_idx]
                _, src_curr_rot = source_current_pose_dict[src_bone]
                src_init_rot = self.src_chain.skeleton.rest_world_rot[src_bone]
            else:
                # Interpolated mode (UE5 Default)
                _, src_curr_rot = self.src_chain.get_interpolated_transform_at_param(
                    tgt_param, source_current_pose_dict, rest_pose=False
                )
                _, src_init_rot = self.src_chain.get_interpolated_transform_at_param(
                    tgt_param, source_current_pose_dict, rest_pose=True
                )

            # Apply parent compensation to source rotation before delta computation
            src_curr_compensated_rot = (parent_offset_rot * src_curr_rot).normal()

            # UE5 Rotation Delta: R_delta = R_src_current * R_src_initial^-1
            rot_delta = (src_curr_compensated_rot * src_init_rot.inverse()).normal()

            # Apply delta to target aligned rest rotation: R_tgt = R_delta * R_tgt_rest
            tgt_rest_rot = self.target_aligned_rot.get(tgt_bone, self.tgt_chain.skeleton.rest_world_rot[tgt_bone])
            final_rot = (rot_delta * tgt_rest_rot).normal()

            solved_rotations[tgt_bone] = final_rot

        return solved_rotations
