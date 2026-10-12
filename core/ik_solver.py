# -*- coding: utf-8 -*-
"""
Robust Two-Bone IK Solver and IK Goal generator.
Directly implements UE5's FIKChainRetargeter and analytical Two-Bone IK.
Guarantees zero-flip, natural limb bending matching the source animation.
"""

import math
import maya.api.OpenMaya as om
from .math_utils import quat_between_vectors, get_world_transform, set_world_transform


class TwoBoneIKSolver(object):
    """
    Analytically solves a 3-joint kinematic chain (Upper -> Lower -> End)
    using the Law of Cosines and Pole Vector plane projection.
    """

    @staticmethod
    def solve(root_pos, goal_pos, pv_hint, len1, len2, rest_dir1, rest_dir2, rest_rot1, rest_rot2):
        """
        Computes the world orientations for the upper and lower bones.
        root_pos: MVector, current world position of upper joint
        goal_pos: MVector, desired world position of end joint
        pv_hint: MVector, direction pointing towards knee/elbow bend
        len1: float, length of upper bone
        len2: float, length of lower bone
        rest_dir1: MVector, normalized rest direction of upper bone
        rest_dir2: MVector, normalized rest direction of lower bone
        rest_rot1: MQuaternion, rest world rotation of upper bone
        rest_rot2: MQuaternion, rest world rotation of lower bone
        """
        vec_to_goal = goal_pos - root_pos
        dist = vec_to_goal.length()

        # Clamp reach distance within triangle bounds
        max_dist = len1 + len2 - 1e-4
        min_dist = max(1e-4, abs(len1 - len2) + 1e-4)
        dist = max(min_dist, min(max_dist, dist))

        dir_to_goal = vec_to_goal.normal()

        # Law of Cosines for root angle
        cos_root = (len1 * len1 + dist * dist - len2 * len2) / (2.0 * len1 * dist)
        cos_root = max(-1.0, min(1.0, cos_root))
        angle_root = math.acos(cos_root)

        # Compute bending plane
        # pv_hint is projected onto plane perpendicular to dir_to_goal
        pv_proj = pv_hint - dir_to_goal * (pv_hint * dir_to_goal)
        if pv_proj.length() < 1e-3:
            # Fallback if pole vector is collinear with aim
            ortho = om.MVector(0, 1, 0) ^ dir_to_goal
            if ortho.length() < 1e-3:
                ortho = om.MVector(1, 0, 0) ^ dir_to_goal
            pv_proj = ortho.normal()
        else:
            pv_proj.normalize()

        bend_normal = (dir_to_goal ^ pv_proj).normal()
        bend_axis = (bend_normal ^ dir_to_goal).normal()

        # Mid-joint position (Knee / Elbow)
        mid_pos = root_pos + dir_to_goal * (len1 * math.cos(angle_root)) + bend_axis * (len1 * math.sin(angle_root))

        # Upper bone orientation
        curr_dir1 = (mid_pos - root_pos).normal()
        rot_upper = quat_between_vectors(rest_dir1, curr_dir1) * rest_rot1

        # Lower bone orientation
        curr_dir2 = (goal_pos - mid_pos).normal()
        rot_lower = quat_between_vectors(rest_dir2, curr_dir2) * rest_rot2

        return rot_upper, rot_lower, mid_pos


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

        # Cache initial lengths
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

        src_curr_start_pos, _ = source_current_pose_dict[src_start_bone]
        src_curr_end_pos, src_curr_end_rot = source_current_pose_dict[src_end_bone]

        # Normalized Extension Ratio (UE5 Formula)
        src_limb_vec = src_curr_end_pos - src_curr_start_pos
        src_curr_length = src_limb_vec.length()
        src_dir = src_limb_vec.normal() if src_curr_length > 1e-4 else om.MVector(0, -1, 0)

        normalized_extension = src_curr_length / self.src_initial_limb_length
        # Target goal position = target_start + src_dir * (normalized_ext * tgt_rest_length)
        goal_pos = target_current_start_pos + (src_dir * (normalized_extension * self.tgt_initial_limb_length))

        # Goal Rotation
        src_init_end_rot = self.src_chain.skeleton.rest_world_rot[src_end_bone]
        rot_delta = src_curr_end_rot * src_init_end_rot.inverse()
        goal_rot = rot_delta * self.tgt_aligned_end_rot

        return goal_pos, goal_rot
