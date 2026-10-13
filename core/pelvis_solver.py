# -*- coding: utf-8 -*-
"""
Pelvis / Root motion retarget solver.
Directly implements UE 5.8's FIKRetargetPelvisMotionOp (PelvisMotionOp.cpp).
"""

import maya.api.OpenMaya as om
from .math_utils import slerp, lerp_vector


class PelvisMotionSolver(object):
    """
    Solves pelvis translation and rotation transferring motion between characters
    with different heights and proportions.
    """

    def __init__(self, source_pelvis, target_pelvis, source_skeleton, target_skeleton, target_pose_mgr=None):
        self.source_pelvis = source_skeleton.resolve_bone_dag(source_pelvis) or source_pelvis
        self.target_pelvis = target_skeleton.resolve_bone_dag(target_pelvis) or target_pelvis
        self.source_skeleton = source_skeleton
        self.target_skeleton = target_skeleton
        self.target_pose_mgr = target_pose_mgr

        # Retargeting parameters (matching UE 5.8 FIKRetargetPelvisMotionOpSettings)
        self.scale_horizontal = 1.0
        self.scale_vertical = 1.0
        self.blend_to_absolute = 0.0  # 0.0 = scaled motion, 1.0 = absolute delta
        self.floor_constraint_weight = 1.0
        self.rotation_alpha = 1.0
        self.affect_ik_horizontal = 1.0
        self.affect_ik_vertical = 1.0

        self.update_retarget_pose()

    def update_retarget_pose(self):
        """Refreshes cached initial pelvis transforms and height ratio."""
        self.source_initial_pos = self.source_skeleton.rest_world_pos[self.source_pelvis]
        self.source_initial_rot = self.source_skeleton.rest_world_rot[self.source_pelvis]

        self.target_initial_pos = (
            self.target_pose_mgr.get_aligned_rest_position(self.target_pelvis)
            if self.target_pose_mgr else self.target_skeleton.rest_world_pos[self.target_pelvis]
        )
        self.target_initial_rot = (
            self.target_pose_mgr.get_aligned_rest_rotation(self.target_pelvis)
            if self.target_pose_mgr else self.target_skeleton.rest_world_rot[self.target_pelvis]
        )

        # Height ratio: Target Pelvis Height / Source Pelvis Height
        src_h = max(1e-3, abs(self.source_initial_pos.y))
        tgt_h = max(1e-3, abs(self.target_initial_pos.y))
        self.height_ratio = tgt_h / src_h
        self.height_scale = self.height_ratio

    def solve(self, source_current_pos, source_current_rot):
        """
        Calculates retargeted target pelvis position and rotation.
        Note on Maya OpenMaya MQuaternion (row-vector left-to-right):
        - World rotation delta: rot_delta = source_initial_rot.inverse() * source_current_rot
        - Apply world delta to target: retargeted_rot = target_initial_rot * rot_delta
        Returns: (target_world_pos, target_world_rot)
        """
        # --- 1. Rotation Retargeting ---
        rot_delta = (self.source_initial_rot.inverse() * source_current_rot).normal()
        retargeted_rot = (self.target_initial_rot * rot_delta).normal()

        if self.rotation_alpha < 1.0 - 1e-4:
            retargeted_rot = slerp(self.target_initial_rot, retargeted_rot, self.rotation_alpha).normal()

        # --- 2. Translation Retargeting ---
        # Scaled Translation:
        norm_y = source_current_pos.y / max(1e-3, self.source_initial_pos.y)
        scaled_y = norm_y * self.target_initial_pos.y

        delta_x = (source_current_pos.x - self.source_initial_pos.x) * self.height_ratio * self.scale_horizontal
        delta_z = (source_current_pos.z - self.source_initial_pos.z) * self.height_ratio * self.scale_horizontal
        pos_scaled = om.MVector(
            self.target_initial_pos.x + delta_x,
            scaled_y * self.scale_vertical,
            self.target_initial_pos.z + delta_z
        )

        # Absolute Translation:
        abs_delta = source_current_pos - self.source_initial_pos
        pos_absolute = self.target_initial_pos + abs_delta

        # Blend between scaled and absolute
        out_pos = lerp_vector(pos_scaled, pos_absolute, self.blend_to_absolute)

        # --- 3. Floor Constraint ---
        if self.floor_constraint_weight > 0.0:
            if source_current_pos.y < self.source_initial_pos.y:
                floor_percent = max(0.0, source_current_pos.y / max(1e-3, self.source_initial_pos.y))
                constrained_y = self.target_initial_pos.y * floor_percent
                out_pos.y = out_pos.y * (1.0 - self.floor_constraint_weight) + constrained_y * self.floor_constraint_weight

        return out_pos, retargeted_rot
