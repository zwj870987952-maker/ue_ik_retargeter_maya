# -*- coding: utf-8 -*-
"""
Retarget Base Pose alignment (e.g. A-Pose to T-Pose matching).
Parallels UE5's FIKRetargetPose and FResolvedRetargetPose.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import get_world_transform, set_world_transform, euler_deg_to_quat


class RetargetPoseManager(object):
    """
    Manages retarget pose alignment offsets without modifying skin bind poses.
    Stores local and global rotation deltas between the character's bind pose
    and an aligned retarget base pose.
    """

    def __init__(self, skeleton):
        self.skeleton = skeleton
        self.delta_rotations = {}  # joint_name -> MQuaternion (offset)
        self.root_translation_offset = om.MVector(0, 0, 0)
        self.is_editing = False
        self._temp_pose_cache = {}

    def start_edit_mode(self):
        """Puts target skeleton in interactive edit mode in the Maya viewport."""
        self.is_editing = True
        self._temp_pose_cache.clear()
        for j in self.skeleton.joint_names:
            pos, rot = get_world_transform(j)
            self._temp_pose_cache[j] = (pos, rot)

    def save_edit_mode(self):
        """
        Saves current joint orientations as retarget alignment pose.
        Computes delta = current_rot * rest_rot^-1.
        """
        self.delta_rotations.clear()
        for j in self.skeleton.joint_names:
            _, curr_rot = get_world_transform(j)
            rest_rot = self.skeleton.rest_world_rot[j]

            # Delta in world/local rotation
            delta = curr_rot * rest_rot.inverse()
            self.delta_rotations[j] = delta

        # Pelvis translation delta
        root_name = self.skeleton.root_joint
        curr_pos, _ = get_world_transform(root_name)
        rest_pos = self.skeleton.rest_world_pos[root_name]
        self.root_translation_offset = curr_pos - rest_pos

        self.is_editing = False

    def reset_to_bind_pose(self):
        """Resets all skeleton joints back to original rest/bind pose."""
        self.delta_rotations.clear()
        self.root_translation_offset = om.MVector(0, 0, 0)
        for j in self.skeleton.joint_names:
            set_world_transform(j, self.skeleton.rest_world_pos[j], self.skeleton.rest_world_rot[j])
        self.is_editing = False

    def get_aligned_rest_rotation(self, joint_name):
        """Returns the rest rotation adjusted by retarget pose alignment delta."""
        base_rot = self.skeleton.rest_world_rot[joint_name]
        if joint_name in self.delta_rotations:
            return self.delta_rotations[joint_name] * base_rot
        return base_rot

    def get_aligned_rest_position(self, joint_name):
        """Returns rest position with retarget pose root offset applied."""
        base_pos = self.skeleton.rest_world_pos[joint_name]
        if joint_name == self.skeleton.root_joint:
            return base_pos + self.root_translation_offset
        return base_pos
