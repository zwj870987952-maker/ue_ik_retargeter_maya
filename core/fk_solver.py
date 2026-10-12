# -*- coding: utf-8 -*-
"""
FK Chain solver.
Directly implements UE5's FChainDecoderFK (FKChainsOp.cpp).
Handles forward kinematics rotation transfer and chain length parameterization.
"""

import maya.api.OpenMaya as om
from .math_utils import slerp


class FKChainSolver(object):
    """
    Retargets rotational poses from a source BoneChain to a target BoneChain.
    Supports OneToOne and Interpolated modes when joint counts differ.
    """

    def __init__(self, mapping, target_pose_mgr=None):
        self.mapping = mapping
        self.src_chain = mapping.source_chain
        self.tgt_chain = mapping.target_chain
        self.target_pose_mgr = target_pose_mgr

        # Cache initial rotations
        self.target_aligned_rot = {}
        for b in self.tgt_chain.bones:
            if self.target_pose_mgr:
                self.target_aligned_rot[b] = self.target_pose_mgr.get_aligned_rest_rotation(b)
            else:
                self.target_aligned_rot[b] = self.tgt_chain.skeleton.rest_world_rot[b]

    def solve(self, source_current_pose_dict):
        """
        Calculates new world rotations for all target joints in this chain.
        Returns: dict of {target_joint_name: target_world_quaternion}
        """
        solved_rotations = {}

        for tgt_idx, tgt_bone in enumerate(self.tgt_chain.bones):
            tgt_param = self.tgt_chain.params[tgt_idx]

            if self.mapping.rotation_mode == "OneToOne":
                # Direct index mapping
                src_idx = min(tgt_idx, len(self.src_chain.bones) - 1)
                src_bone = self.src_chain.bones[src_idx]
                _, src_curr_rot = source_current_pose_dict[src_bone]
                src_init_rot = self.src_chain.skeleton.rest_world_rot[src_bone]
            else:
                # Interpolated mode (UE5 Default): sample source chain at target param
                _, src_curr_rot = self.src_chain.get_interpolated_transform_at_param(
                    tgt_param, source_current_pose_dict, rest_pose=False
                )
                _, src_init_rot = self.src_chain.get_interpolated_transform_at_param(
                    tgt_param, source_current_pose_dict, rest_pose=True
                )

            # Delta: R_delta = R_src_current * R_src_initial^-1
            rot_delta = src_curr_rot * src_init_rot.inverse()

            # Apply delta to target aligned rest rotation: R_tgt = R_delta * R_tgt_rest
            tgt_rest_rot = self.target_aligned_rot[tgt_bone]
            final_rot = rot_delta * tgt_rest_rot
            solved_rotations[tgt_bone] = final_rot

        return solved_rotations
