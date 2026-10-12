# -*- coding: utf-8 -*-
"""
Robust Two-Bone IK Solver and IK Goal generator.
Directly implements UE5's FIKChainRetargeter and AnimationCore::SolveTwoBoneIK.
Preserves full FK axial roll/twist on limbs and ensures natural, un-twisted hands and feet.
"""

import math
import maya.api.OpenMaya as om
from .math_utils import quat_between_vectors, slerp


class TwoBoneIKSolver(object):
    """
    Analytically solves a 3-joint kinematic chain (Upper -> Lower -> End).
    Directly ports UE5's AnimationCore::SolveTwoBoneIK (TwoBoneIK.cpp:26-60, 70-185).
    Preserves full FK axial roll/twist on upper and lower limbs via DeltaRotation multiplication.
    """

    @staticmethod
    def solve(
        root_pos, joint_pos, end_pos,
        root_rot, joint_rot, end_rot,
        effector_pos, effector_rot,
        joint_target_hint,
        upper_len, lower_len,
        end_rot_blend=0.0
    ):
        """
        Solves Two-Bone IK matching UE5 AnimationCore::SolveTwoBoneIK.
        root_pos, joint_pos, end_pos: MVectors of current FK positions.
        root_rot, joint_rot, end_rot: MQuaternions of current FK rotations.
        effector_pos: MVector, desired world position of end joint.
        effector_rot: MQuaternion, desired world rotation of end joint (IK Goal).
        joint_target_hint: MVector, pole vector target position in world space.
        upper_len: float, length of upper limb.
        lower_len: float, length of lower limb.
        end_rot_blend: float, 0.0 = keep FK relative wrist/ankle orient, 1.0 = match effector_rot.
        """
        desired_pos = effector_pos
        desired_delta = desired_pos - root_pos
        desired_length = desired_delta.length()
        max_limb_length = upper_len + lower_len

        if desired_length < 1e-4:
            desired_length = 1e-4
            desired_dir = om.MVector(1.0, 0.0, 0.0)
        else:
            desired_dir = desired_delta.normal()

        # Get joint target (bend plane hint)
        if joint_target_hint is not None:
            joint_target_delta = joint_target_hint - root_pos
        else:
            joint_target_delta = joint_pos - root_pos

        joint_plane_normal = desired_dir ^ joint_target_delta
        if joint_plane_normal.length() < 1e-4:
            # Fallback to current joint pos delta
            joint_target_delta = joint_pos - root_pos
            joint_plane_normal = desired_dir ^ joint_target_delta

        if joint_plane_normal.length() < 1e-4:
            # Limb is completely straight: find orthogonal vector
            ortho = om.MVector(0.0, 1.0, 0.0) ^ desired_dir
            if ortho.length() < 1e-3:
                ortho = om.MVector(1.0, 0.0, 0.0) ^ desired_dir
            joint_plane_normal = ortho.normal()
            joint_bend_dir = (joint_plane_normal ^ desired_dir).normal()
        else:
            joint_plane_normal.normalize()
            # Remove component along desired_dir to get pure bend direction
            joint_bend_dir = joint_target_delta - desired_dir * (joint_target_delta * desired_dir)
            if joint_bend_dir.length() < 1e-4:
                joint_bend_dir = (joint_plane_normal ^ desired_dir).normal()
            else:
                joint_bend_dir.normalize()

        # Clamping reach beyond max limb length (UE5 formula)
        if desired_length >= max_limb_length:
            out_end_pos = root_pos + desired_dir * max_limb_length
            out_joint_pos = root_pos + desired_dir * upper_len
        else:
            two_ab = 2.0 * upper_len * desired_length
            cos_angle = ((upper_len * upper_len) + (desired_length * desired_length) - (lower_len * lower_len)) / two_ab if two_ab > 1e-6 else 0.0
            cos_angle = max(-1.0, min(1.0, cos_angle))
            b_reverse = (cos_angle < 0.0)
            angle = math.acos(cos_angle)

            joint_line_dist = upper_len * math.sin(angle)
            proj_joint_dist_sqr = (upper_len * upper_len) - (joint_line_dist * joint_line_dist)
            proj_joint_dist = math.sqrt(max(0.0, proj_joint_dist_sqr))
            if b_reverse:
                proj_joint_dist = -proj_joint_dist

            out_joint_pos = root_pos + (desired_dir * proj_joint_dist) + (joint_bend_dir * joint_line_dist)
            out_end_pos = desired_pos

        # UE5 TwoBoneIK Delta Rotations (Lines 26-53):
        # Upper bone
        old_dir_upper = (joint_pos - root_pos).normal()
        new_dir_upper = (out_joint_pos - root_pos).normal()
        delta_rot_upper = quat_between_vectors(old_dir_upper, new_dir_upper)
        solved_root_rot = (delta_rot_upper * root_rot).normal()

        # Middle bone
        old_dir_mid = (end_pos - joint_pos).normal()
        new_dir_mid = (out_end_pos - out_joint_pos).normal()
        delta_rot_mid = quat_between_vectors(old_dir_mid, new_dir_mid)
        solved_joint_rot = (delta_rot_mid * joint_rot).normal()

        # End bone (Hand / Foot):
        # In FK, end bone rotation naturally tracks delta_rot_mid
        fk_tracked_end_rot = (delta_rot_mid * end_rot).normal()
        if effector_rot is not None and end_rot_blend > 1e-4:
            solved_end_rot = slerp(fk_tracked_end_rot, effector_rot, end_rot_blend).normal()
        else:
            solved_end_rot = fk_tracked_end_rot

        return solved_root_rot, solved_joint_rot, solved_end_rot, out_joint_pos, out_end_pos


class IKChainSolver(object):
    """
    Computes IK goal positions based on source limb normalized extension,
    and solves target limb joint rotations to hit the goal precisely.
    Matches UE5's FIKChainRetargeter (IKChainsOp.cpp).
    """

    def __init__(self, mapping, target_pose_mgr=None):
        self.mapping = mapping
        self.src_chain = mapping.source_chain
        self.tgt_chain = mapping.target_chain
        self.target_pose_mgr = target_pose_mgr

        self.update_retarget_pose()

    def update_retarget_pose(self):
        """Refreshes rest limb lengths and aligned end rotation."""
        src_start_pos = self.src_chain.skeleton.rest_world_pos[self.src_chain.start_bone]
        src_end_pos = self.src_chain.skeleton.rest_world_pos[self.src_chain.end_bone]
        self.src_initial_limb_length = max(1e-4, (src_end_pos - src_start_pos).length())

        if self.target_pose_mgr:
            tgt_start_pos = self.target_pose_mgr.get_aligned_rest_position(self.tgt_chain.start_bone)
            tgt_end_pos = self.target_pose_mgr.get_aligned_rest_position(self.tgt_chain.end_bone)
            self.tgt_aligned_end_rot = self.target_pose_mgr.get_aligned_rest_rotation(self.tgt_chain.end_bone)
        else:
            tgt_start_pos = self.tgt_chain.skeleton.rest_world_pos[self.tgt_chain.start_bone]
            tgt_end_pos = self.tgt_chain.skeleton.rest_world_pos[self.tgt_chain.end_bone]
            self.tgt_aligned_end_rot = self.tgt_chain.skeleton.rest_world_rot[self.tgt_chain.end_bone]

        self.tgt_initial_limb_length = max(1e-4, (tgt_end_pos - tgt_start_pos).length())

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

        # Normalized Extension Ratio (UE5 Formula from IKChainsOp.cpp)
        src_limb_vec = src_curr_end_pos - src_curr_start_pos
        src_curr_length = src_limb_vec.length()
        src_dir = src_limb_vec.normal() if src_curr_length > 1e-4 else om.MVector(0, -1, 0)

        normalized_extension = src_curr_length / self.src_initial_limb_length
        # Target goal position = target_start + src_dir * (normalized_ext * tgt_rest_length)
        goal_pos = target_current_start_pos + (src_dir * (normalized_extension * self.tgt_initial_limb_length))

        # Goal Rotation:
        # rot_delta = CurrentSourceEndRotation * InitialSourceEndRotation^-1
        # goal_rot = rot_delta * InitialTargetEndRotation
        src_init_end_rot = self.src_chain.skeleton.rest_world_rot[src_end_bone]
        rot_delta = (src_curr_end_rot * src_init_end_rot.inverse()).normal()

        if self.target_pose_mgr:
            aligned_end_rot = self.target_pose_mgr.get_aligned_rest_rotation(tgt_end_bone)
        else:
            aligned_end_rot = self.tgt_aligned_end_rot

        goal_rot = (rot_delta * aligned_end_rot).normal()

        return goal_pos, goal_rot
