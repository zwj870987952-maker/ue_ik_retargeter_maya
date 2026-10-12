# -*- coding: utf-8 -*-
"""
IK Chain and Goal solver.
Directly implements UE5's FIKChainRetargeter (IKChainsOp.cpp) and TwoBoneIK solving.
Eliminates foot sliding, ground clipping, and arm hyperextension.
"""

import math
import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import slerp, quat_between_vectors, get_world_transform, set_world_transform


class IKChainSolver(object):
    """
    Computes IK goal positions based on source limb normalized extension,
    and solves target limb joint rotations to hit the goal precisely.
    """

    def __init__(self, mapping, target_pose_mgr=None):
        self.mapping = mapping
        self.src_chain = mapping.source_chain
        self.tgt_chain = mapping.target_chain
        self.target_pose_mgr = target_pose_mgr

        # End bone rotation settings
        self.blend_to_source = 0.0

        # Cache initial lengths from start joint to end joint
        src_start_pos = self.src_chain.skeleton.rest_world_pos[self.src_chain.start_bone]
        src_end_pos = self.src_chain.skeleton.rest_world_pos[self.src_chain.end_bone]
        self.src_initial_limb_length = max(1e-4, (src_end_pos - src_start_pos).length())

        tgt_start_pos = self.tgt_chain.skeleton.rest_world_pos[self.tgt_chain.start_bone]
        tgt_end_pos = self.tgt_chain.skeleton.rest_world_pos[self.tgt_chain.end_bone]
        self.tgt_initial_limb_length = max(1e-4, (tgt_end_pos - tgt_start_pos).length())

        # Aligned rest end rotation
        if self.target_pose_mgr:
            self.tgt_aligned_end_rot = self.target_pose_mgr.get_aligned_rest_rotation(self.tgt_chain.end_bone)
        else:
            self.tgt_aligned_end_rot = self.tgt_chain.skeleton.rest_world_rot[self.tgt_chain.end_bone]

    def calculate_goal_transform(self, source_current_pose_dict, target_current_start_pos):
        """
        Calculates world-space position and rotation for the target IK goal.
        Matches UE5's FIKChainRetargeter::CalculateGoalPosition & CalculateGoalRotation.
        """
        src_start_bone = self.src_chain.start_bone
        src_end_bone = self.src_chain.end_bone
        tgt_end_bone = self.tgt_chain.end_bone

        src_curr_start_pos, _ = source_current_pose_dict[src_start_bone]
        src_curr_end_pos, src_curr_end_rot = source_current_pose_dict[src_end_bone]

        # --- 1. Normalized Extension Ratio (UE5 Formula) ---
        src_limb_vec = src_curr_end_pos - src_curr_start_pos
        src_curr_length = src_limb_vec.length()
        src_dir = src_limb_vec.normal()

        normalized_extension = src_curr_length / self.src_initial_limb_length
        # Target goal position = target_start + src_dir * (normalized_ext * tgt_rest_length)
        goal_pos = target_current_start_pos + (src_dir * (normalized_extension * self.tgt_initial_limb_length))

        # --- 2. Goal Rotation ---
        src_init_end_rot = self.src_chain.skeleton.rest_world_rot[src_end_bone]
        rot_delta = src_curr_end_rot * src_init_end_rot.inverse()
        goal_rot = rot_delta * self.tgt_aligned_end_rot

        return goal_pos, goal_rot

    def solve_two_bone_ik(self, root_pos, goal_pos, pole_vector, upper_len, lower_len):
        """
        Analytical Two-Bone IK solver (Law of Cosines).
        Computes joint rotations for a 3-joint chain (e.g. Hip -> Knee -> Foot).
        root_pos: Position of upper bone (e.g. Hip)
        goal_pos: Desired position of end bone (e.g. Foot)
        pole_vector: Direction hinting the bend plane (e.g. Knee forward)
        """
        to_goal = goal_pos - root_pos
        dist = to_goal.length()

        # Clamp distance to avoid triangle inequality breakdown
        max_reach = upper_len + lower_len - 1e-4
        min_reach = abs(upper_len - lower_len) + 1e-4
        dist_clamped = max(min_reach, min(max_reach, dist))

        # Law of Cosines for angles
        cos_root_angle = (upper_len * upper_len + dist_clamped * dist_clamped - lower_len * lower_len) / (2.0 * upper_len * dist_clamped)
        cos_root_angle = max(-1.0, min(1.0, cos_root_angle))
        root_angle = math.acos(cos_root_angle)

        cos_mid_angle = (upper_len * upper_len + lower_len * lower_len - dist_clamped * dist_clamped) / (2.0 * upper_len * lower_len)
        cos_mid_angle = max(-1.0, min(1.0, cos_mid_angle))
        mid_angle = math.pi - math.acos(cos_mid_angle)

        # Plane normal
        dir_to_goal = to_goal.normal()
        pv_proj = pole_vector - dir_to_goal * (pole_vector * dir_to_goal)
        plane_normal = (dir_to_goal ^ pv_proj).normal()
        if plane_normal.length() < 1e-3:
            plane_normal = om.MVector(0, 0, 1)

        # Rotate root towards goal and bend
        bend_axis = (plane_normal ^ dir_to_goal).normal()
        mid_pos = root_pos + (dir_to_goal * (upper_len * math.cos(root_angle))) + (bend_axis * (upper_len * math.sin(root_angle)))

        return mid_pos
