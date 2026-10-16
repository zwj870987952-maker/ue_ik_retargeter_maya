# -*- coding: utf-8 -*-
"""
Pelvis / Root motion retarget solver.
Directly implements UE 5.8's FIKRetargetPelvisMotionOp (PelvisMotionOp.cpp).
"""

import maya.api.OpenMaya as om
from .math_utils import slerp, lerp_vector, rotate_vector_by_quat


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
        self.translation_alpha = 1.0
        self.source_crotch_offset = 0.0
        self.target_crotch_offset = 0.0
        self.use_ground_falloff = False
        self.ground_falloff_height_percent = .9
        self.blend_to_source_translation = 0.0
        self.blend_to_source_translation_weights = [1.,1.,1.]
        self.translation_offset_local = [0.,0.,0.]
        self.translation_offset_global = [0.,0.,0.]
        self.rotation_offset_local = [0.,0.,0.,1.]
        self.rotation_offset_global = [0.,0.,0.,1.]
        self.pelvis_translation_delta = om.MVector()

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
        src_h = self.source_initial_pos.y if self.source_initial_pos.y > 1e-4 else 1.
        tgt_h = self.target_initial_pos.y
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
        retargeted_rot = (om.MQuaternion(*self.rotation_offset_local)*retargeted_rot*om.MQuaternion(*self.rotation_offset_global)).normal()

        if self.rotation_alpha < 1.0 - 1e-4:
            retargeted_rot = slerp(self.target_initial_rot, retargeted_rot, self.rotation_alpha).normal()

        # --- 2. Translation Retargeting ---
        # Scaled Translation:
        pos_scaled = source_current_pos * self.height_ratio

        # Absolute Translation:
        abs_delta = source_current_pos - self.source_initial_pos
        pos_absolute = self.target_initial_pos + abs_delta

        # Blend between scaled and absolute
        weights=self.blend_to_absolute
        if not isinstance(weights,(list,tuple)):weights=[weights]*3
        out_pos=om.MVector(*[a+(b-a)*w for a,b,w in zip(pos_scaled,pos_absolute,weights)])

        # --- 3. Floor Constraint ---
        if self.floor_constraint_weight > 0.0:
            if source_current_pos.y <= self.source_initial_pos.y:
                floor_percent = (source_current_pos.y-self.source_crotch_offset)/max(1.,self.source_initial_pos.y-self.source_crotch_offset)
                constrained_y = (self.target_initial_pos.y-self.target_crotch_offset)*floor_percent+self.target_crotch_offset
                weight=self.floor_constraint_weight
                if self.use_ground_falloff:
                    falloff=self.target_initial_pos.y*self.ground_falloff_height_percent
                    weight*=1.-max(0.,min(out_pos.y,falloff))/max(1e-6,falloff)
                out_pos.y += (constrained_y-out_pos.y)*weight

        # UE records this BEFORE user adjustments. Affect IK subtracts only
        # adjustment deltas, never the pelvis motion already retargeted above.
        unmodified=om.MVector(out_pos)
        out_pos=om.MVector(*[a+(b-a)*self.blend_to_source_translation*w for a,b,w in zip(out_pos,source_current_pos,self.blend_to_source_translation_weights)])
        out_pos.y*=self.scale_vertical
        offset=out_pos-self.target_initial_pos
        out_pos=self.target_initial_pos+om.MVector(offset.x*self.scale_horizontal,offset.y,offset.z*self.scale_horizontal)
        out_pos+=om.MVector(*self.translation_offset_global)+rotate_vector_by_quat(om.MVector(*self.translation_offset_local),retargeted_rot)
        out_pos=lerp_vector(self.target_initial_pos,out_pos,self.translation_alpha)
        self.pelvis_translation_delta=out_pos-unmodified

        return out_pos, retargeted_rot
