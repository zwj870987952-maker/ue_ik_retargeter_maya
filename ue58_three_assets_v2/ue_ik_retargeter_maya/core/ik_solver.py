# -*- coding: utf-8 -*-
"""
Robust Two-Bone IK Solver and IK Goal generator.
Directly implements UE 5.8's FIKChainRetargeter (IKChainsOp.cpp),
RunIKRigOp (RunIKRigOp.cpp), and AnimationCore::SolveTwoBoneIK / FLimbSolver (LimbSolver.cpp).
Implements InvRootModification (ground locking) and VectorPlaneProject pole vector alignment.
"""

import math
import maya.api.OpenMaya as om
from .math_utils import quat_between_vectors, slerp, rotate_vector_by_quat


class TwoBoneIKSolver(object):
    """
    Analytically solves a 3-joint kinematic chain (Upper -> Lower -> End).
    Directly ports UE 5.8's FLimbSolver::Solve & AnimationCore::SolveTwoBoneIK.
    Preserves full FK axial roll/twist on upper and lower limbs via DeltaRotation.
    """

    @staticmethod
    def solve(
        root_pos, joint_pos, end_pos,
        root_rot, joint_rot, end_rot,
        effector_pos, effector_rot,
        joint_target_hint,
        upper_len, lower_len,
        end_rot_blend=1.0
    ):
        """
        Solves Two-Bone IK matching UE 5.8 AnimationCore::SolveTwoBoneIK.
        Note on Maya OpenMaya MQuaternion (row-vector left-to-right):
        Applying a world-space rotation delta `delta_rot` to a bone rotation `rot` is `rot * delta_rot`.
        """
        desired_pos = effector_pos
        desired_delta = desired_pos - root_pos
        desired_length = desired_delta.length()
        max_limb_length = upper_len + lower_len

        if desired_length < 1e-4:
            desired_length = 1e-4
            desired_dir = om.MVector(0.0, -1.0, 0.0)
        else:
            desired_dir = desired_delta.normal()

        # 1. Determine Bend Direction (UE 5.8 VectorPlaneProject logic)
        fk_upper_dir = (joint_pos - root_pos).normal() if (joint_pos - root_pos).length() > 1e-4 else desired_dir
        fk_plane_normal = desired_dir ^ fk_upper_dir

        if joint_target_hint is not None and (joint_target_hint - root_pos).length() > 1e-4:
            hint_dir = (joint_target_hint - root_pos).normal()
            proj_hint = hint_dir - desired_dir * (hint_dir * desired_dir)
            if proj_hint.length() > 1e-4:
                joint_bend_dir = proj_hint.normal()
            else:
                joint_bend_dir = None
        else:
            joint_bend_dir = None

        if joint_bend_dir is None:
            if fk_plane_normal.length() > 1e-4:
                joint_bend_dir = (fk_plane_normal.normal() ^ desired_dir).normal()
            else:
                ortho = om.MVector(0.0, 0.0, 1.0) ^ desired_dir
                if ortho.length() < 1e-3:
                    ortho = om.MVector(1.0, 0.0, 0.0) ^ desired_dir
                joint_bend_dir = ortho.normal()

        # 2. Clamping reach beyond max limb length (UE 5.8 formula)
        if desired_length >= max_limb_length * 0.9999:
            out_end_pos = root_pos + desired_dir * max_limb_length
            out_joint_pos = root_pos + desired_dir * upper_len
        else:
            two_ab = 2.0 * upper_len * desired_length
            cos_angle = ((upper_len * upper_len) + (desired_length * desired_length) - (lower_len * lower_len)) / two_ab if two_ab > 1e-6 else 0.0
            cos_angle = max(-1.0, min(1.0, cos_angle))
            angle = math.acos(cos_angle)

            proj_dist = upper_len * cos_angle
            joint_line_dist = upper_len * math.sin(angle)

            out_joint_pos = root_pos + (desired_dir * proj_dist) + (joint_bend_dir * joint_line_dist)
            out_end_pos = desired_pos

        # 3. UE 5.8 Delta Rotations for Upper and Middle bones (row-vector: rot * delta_rot)
        old_dir_upper = (joint_pos - root_pos).normal() if (joint_pos - root_pos).length() > 1e-4 else desired_dir
        new_dir_upper = (out_joint_pos - root_pos).normal()
        delta_rot_upper = quat_between_vectors(old_dir_upper, new_dir_upper)
        solved_root_rot = (root_rot * delta_rot_upper).normal()

        # Because Middle joint is a child of Upper joint, rotating Upper by delta_rot_upper
        # rotates Middle's world orientation and its limb vector by delta_rot_upper first:
        raw_old_dir_mid = (end_pos - joint_pos).normal() if (end_pos - joint_pos).length() > 1e-4 else desired_dir
        old_dir_mid = rotate_vector_by_quat(raw_old_dir_mid, delta_rot_upper).normal()
        new_dir_mid = (out_end_pos - out_joint_pos).normal()
        delta_rot_mid = quat_between_vectors(old_dir_mid, new_dir_mid)
        joint_rot_after_upper = (joint_rot * delta_rot_upper).normal()
        solved_joint_rot = (joint_rot_after_upper * delta_rot_mid).normal()

        # 4. End bone (Foot / Hand) rotation
        fk_tracked_end_rot = (end_rot * delta_rot_upper * delta_rot_mid).normal()
        if effector_rot is not None and end_rot_blend > 1e-4:
            solved_end_rot = slerp(fk_tracked_end_rot, effector_rot, end_rot_blend).normal()
        else:
            solved_end_rot = fk_tracked_end_rot

        return solved_root_rot, solved_joint_rot, solved_end_rot, out_joint_pos, out_end_pos


class IKChainSolver(object):
    """
    Computes IK goal positions based on source limb normalized extension,
    and solves target limb joint rotations to hit the goal precisely.
    Implements UE 5.8's FIKChainRetargeter (IKChainsOp.cpp) and InvRootModification.
    """

    def __init__(self, mapping, target_pose_mgr=None, source_pose_mgr=None):
        self.mapping = mapping
        self.src_chain = mapping.source_chain
        self.tgt_chain = mapping.target_chain
        self.target_pose_mgr = target_pose_mgr
        self.source_pose_mgr = source_pose_mgr

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

    def calculate_goal_transform(self, source_current_pose_dict, target_current_start_pos, pelvis_translation_delta=None, affect_ik_vertical=True):
        """
        Calculates world-space position and rotation for the target IK goal.
        Matches UE 5.8's FIKChainRetargeter & RunIKRigOp lines 120-136.
        """
        src_start_bone = self.src_chain.start_bone
        src_end_bone = self.src_chain.end_bone
        tgt_end_bone = self.tgt_chain.end_bone

        src_curr_start_pos, _ = source_current_pose_dict[src_start_bone]
        src_curr_end_pos, src_curr_end_rot = source_current_pose_dict[src_end_bone]

        # 1. Normalized Extension Ratio (UE 5.8 IKChainsOp.cpp lines 120-126)
        src_limb_vec = src_curr_end_pos - src_curr_start_pos
        src_curr_length = src_limb_vec.length()
        src_dir = src_limb_vec.normal() if src_curr_length > 1e-4 else om.MVector(0, -1, 0)

        normalized_extension = src_curr_length / self.src_initial_limb_length
        src_end_dir_normalized = src_dir * normalized_extension

        # 2. InvRootModification (UE 5.8 RunIKRigOp.cpp lines 127-133)
        inv_root_modification = om.MVector(0, 0, 0)
        if not affect_ik_vertical and pelvis_translation_delta is not None:
            inv_root_modification = om.MVector(0.0, pelvis_translation_delta.y, 0.0)

        adjusted_target_start = target_current_start_pos - inv_root_modification

        # Goal Position = AdjustedStart + (NormalizedDirection * InitialTargetLength)
        goal_pos = adjusted_target_start + (src_end_dir_normalized * self.tgt_initial_limb_length)

        # 3. Goal Rotation (UE 5.8 IKChainsOp.cpp lines 89-93, Maya row-vector order)
        src_init_end_rot = self.src_chain.skeleton.rest_world_rot[src_end_bone]
        rot_delta = (src_init_end_rot.inverse() * src_curr_end_rot).normal()

        if self.target_pose_mgr:
            aligned_end_rot = self.target_pose_mgr.get_aligned_rest_rotation(tgt_end_bone)
        else:
            aligned_end_rot = self.tgt_aligned_end_rot

        goal_rot = (aligned_end_rot * rot_delta).normal()

        return goal_pos, goal_rot
