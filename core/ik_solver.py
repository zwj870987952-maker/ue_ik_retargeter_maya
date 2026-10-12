# -*- coding: utf-8 -*-
"""
Robust Two-Bone IK Solver and IK Goal generator.
Directly implements UE5's FIKChainRetargeter (IKChainsOp.cpp),
RunIKRigOp (RunIKRigOp.cpp), and AnimationCore::SolveTwoBoneIK / FLimbSolver (LimbSolver.cpp).
Implements InvRootModification (ground locking) and VectorPlaneProject pole vector alignment.
"""

import math
import maya.api.OpenMaya as om
from .math_utils import quat_between_vectors, slerp, rotate_vector_by_quat


class TwoBoneIKSolver(object):
    """
    Analytically solves a 3-joint kinematic chain (Upper -> Lower -> End).
    Directly ports UE5's FLimbSolver::Solve & AnimationCore::SolveTwoBoneIK.
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
        Solves Two-Bone IK matching UE5 AnimationCore::SolveTwoBoneIK.
        root_pos, joint_pos, end_pos: MVectors of current FK positions.
        root_rot, joint_rot, end_rot: MQuaternions of current FK rotations.
        effector_pos: MVector, desired world position of end joint (IK Goal).
        effector_rot: MQuaternion, desired world rotation of end joint (IK Goal).
        joint_target_hint: MVector, pole vector target direction / hint.
        upper_len: float, length of upper limb.
        lower_len: float, length of lower limb.
        end_rot_blend: float, 1.0 = match effector_rot (feet), 0.0 = track mid bone delta (hands).
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

        # 1. Determine Bend Direction (UE5 VectorPlaneProject logic)
        # First check current FK bend direction
        fk_upper_dir = (joint_pos - root_pos).normal() if (joint_pos - root_pos).length() > 1e-4 else desired_dir
        fk_plane_normal = desired_dir ^ fk_upper_dir

        if joint_target_hint is not None and (joint_target_hint - root_pos).length() > 1e-4:
            hint_dir = (joint_target_hint - root_pos).normal()
            # Project hint onto plane perpendicular to desired_dir
            proj_hint = hint_dir - desired_dir * (hint_dir * desired_dir)
            if proj_hint.length() > 1e-4:
                joint_bend_dir = proj_hint.normal()
            else:
                joint_bend_dir = None
        else:
            joint_bend_dir = None

        if joint_bend_dir is None:
            if fk_plane_normal.length() > 1e-4:
                # Perpendicular to desired_dir in the FK triangle plane
                joint_bend_dir = (fk_plane_normal.normal() ^ desired_dir).normal()
            else:
                # Limb is completely straight: find orthogonal fallback
                ortho = om.MVector(0.0, 0.0, 1.0) ^ desired_dir
                if ortho.length() < 1e-3:
                    ortho = om.MVector(1.0, 0.0, 0.0) ^ desired_dir
                joint_bend_dir = ortho.normal()

        # 2. Clamping reach beyond max limb length (UE5 formula)
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

        # 3. UE5 Delta Rotations for Upper and Middle bones
        old_dir_upper = (joint_pos - root_pos).normal() if (joint_pos - root_pos).length() > 1e-4 else desired_dir
        new_dir_upper = (out_joint_pos - root_pos).normal()
        delta_rot_upper = quat_between_vectors(old_dir_upper, new_dir_upper)
        solved_root_rot = (delta_rot_upper * root_rot).normal()

        old_dir_mid = (end_pos - joint_pos).normal() if (end_pos - joint_pos).length() > 1e-4 else desired_dir
        new_dir_mid = (out_end_pos - out_joint_pos).normal()
        delta_rot_mid = quat_between_vectors(old_dir_mid, new_dir_mid)
        solved_joint_rot = (delta_rot_mid * joint_rot).normal()

        # 4. End bone (Foot / Hand) rotation
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
    Implements UE5's FIKChainRetargeter (IKChainsOp.cpp) and InvRootModification.
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

    def calculate_goal_transform(self, source_current_pose_dict, target_current_start_pos, pelvis_translation_delta=None, affect_ik_vertical=True):
        """
        Calculates world-space position and rotation for the target IK goal.
        Matches UE5's FIKChainRetargeter & RunIKRigOp lines 120-136:
        - Normalized limb extension ratio
        - InvRootModification (pelvis vertical movement isolation for ground pinning when affect_ik_vertical is False)
        """
        src_start_bone = self.src_chain.start_bone
        src_end_bone = self.src_chain.end_bone
        tgt_end_bone = self.tgt_chain.end_bone

        src_curr_start_pos, _ = source_current_pose_dict[src_start_bone]
        src_curr_end_pos, src_curr_end_rot = source_current_pose_dict[src_end_bone]

        # 1. Normalized Extension Ratio (UE5 IKChainsOp.cpp lines 120-126)
        src_limb_vec = src_curr_end_pos - src_curr_start_pos
        src_curr_length = src_limb_vec.length()
        src_dir = src_limb_vec.normal() if src_curr_length > 1e-4 else om.MVector(0, -1, 0)

        normalized_extension = src_curr_length / self.src_initial_limb_length
        src_end_dir_normalized = src_dir * normalized_extension

        # 2. InvRootModification (UE5 RunIKRigOp.cpp lines 127-133)
        # Default in UE5 is affect_ik_vertical=True (pelvis translates IK goals naturally)
        inv_root_modification = om.MVector(0, 0, 0)
        if not affect_ik_vertical and pelvis_translation_delta is not None:
            # Only isolate vertical pelvis movement from pulling IK goals when specifically requested
            inv_root_modification = om.MVector(0.0, pelvis_translation_delta.y, 0.0)

        adjusted_target_start = target_current_start_pos - inv_root_modification

        # Goal Position = AdjustedStart + (NormalizedDirection * InitialTargetLength)
        goal_pos = adjusted_target_start + (src_end_dir_normalized * self.tgt_initial_limb_length)

        # 3. Goal Rotation (UE5 IKChainsOp.cpp lines 89-93)
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
