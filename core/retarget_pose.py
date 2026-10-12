# -*- coding: utf-8 -*-
"""
Retarget Base Pose alignment (e.g. A-Pose to T-Pose matching).
Parallels UE5's FIKRetargetPose and FResolvedRetargetPose.
Properly propagates alignment rotations down the full skeletal hierarchy.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import get_world_transform, set_world_transform, euler_deg_to_quat, rotate_vector_by_quat


class RetargetPoseManager(object):
    """
    Manages retarget pose alignment offsets without modifying skin bind poses.
    Stores local and global rotation deltas between the character's bind pose
    and an aligned retarget base pose.
    Propagates parent rotations to all descendant bones in the hierarchy,
    matching UE5's FRetargetSkeleton::UpdateGlobalTransformsBelowBone.
    """

    def __init__(self, skeleton):
        self.skeleton = skeleton
        self.delta_rotations = {}  # joint_name -> MQuaternion (offset)
        self.root_translation_offset = om.MVector(0, 0, 0)
        self.aligned_world_pos = {}
        self.aligned_world_rot = {}
        self.is_editing = False
        self._temp_pose_cache = {}

        self.recompute_aligned_transforms()

    def recompute_aligned_transforms(self):
        """
        Recomputes aligned world transforms for all joints in topological order.
        Properly propagates alignment rotations down the skeletal hierarchy,
        ensuring children (forearms, hands, knees, feet) follow parent alignments.
        Matches UE5's FRetargetSkeleton::UpdateGlobalTransformsBelowBone.
        """
        self.aligned_world_pos = {}
        self.aligned_world_rot = {}

        for idx, j_name in enumerate(self.skeleton.joint_names):
            p_idx = self.skeleton.parent_indices[idx]
            rest_pos = self.skeleton.rest_world_pos[j_name]
            rest_rot = self.skeleton.rest_world_rot[j_name]

            if p_idx == -1:
                # Root joint
                aligned_pos = rest_pos + self.root_translation_offset
                delta_rot = self.delta_rotations.get(j_name, om.MQuaternion.kIdentity)
                aligned_rot = delta_rot * rest_rot
            else:
                p_name = self.skeleton.joint_names[p_idx]
                p_rest_pos = self.skeleton.rest_world_pos[p_name]
                p_rest_rot = self.skeleton.rest_world_rot[p_name]

                p_aligned_pos = self.aligned_world_pos[p_name]
                p_aligned_rot = self.aligned_world_rot[p_name]

                # Child rest transform relative to parent rest transform:
                # local translation vector in parent frame
                local_vec = rotate_vector_by_quat(rest_pos - p_rest_pos, p_rest_rot.inverse())
                # local rotation in parent frame
                local_rot = p_rest_rot.inverse() * rest_rot

                # Propagated world position
                aligned_pos = p_aligned_pos + rotate_vector_by_quat(local_vec, p_aligned_rot)

                # Propagated world rotation
                propagated_rot = p_aligned_rot * local_rot

                # If joint itself has an explicit alignment delta, apply it
                child_delta = self.delta_rotations.get(j_name, om.MQuaternion.kIdentity)
                aligned_rot = child_delta * propagated_rot

            self.aligned_world_pos[j_name] = aligned_pos
            self.aligned_world_rot[j_name] = aligned_rot

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

            # Delta in world rotation
            delta = curr_rot * rest_rot.inverse()
            if not (abs(delta.x) < 1e-5 and abs(delta.y) < 1e-5 and abs(delta.z) < 1e-5 and abs(delta.w - 1.0) < 1e-5):
                self.delta_rotations[j] = delta

        # Pelvis translation delta
        root_name = self.skeleton.root_joint
        curr_pos, _ = get_world_transform(root_name)
        rest_pos = self.skeleton.rest_world_pos[root_name]
        self.root_translation_offset = curr_pos - rest_pos

        self.recompute_aligned_transforms()
        self.is_editing = False

    def reset_to_bind_pose(self):
        """Resets all skeleton joints back to original rest/bind pose."""
        self.delta_rotations.clear()
        self.root_translation_offset = om.MVector(0, 0, 0)
        self.recompute_aligned_transforms()
        for j in self.skeleton.joint_names:
            set_world_transform(j, self.skeleton.rest_world_pos[j], self.skeleton.rest_world_rot[j])
        self.is_editing = False

    def get_aligned_rest_rotation(self, joint_name):
        """Returns the rest rotation adjusted by retarget pose alignment delta and hierarchy propagation."""
        if joint_name in self.aligned_world_rot:
            return self.aligned_world_rot[joint_name]
        return self.skeleton.rest_world_rot.get(joint_name, om.MQuaternion.kIdentity)

    def get_aligned_rest_position(self, joint_name):
        """Returns rest position with retarget pose root offset and hierarchy propagation applied."""
        if joint_name in self.aligned_world_pos:
            return self.aligned_world_pos[joint_name]
        return self.skeleton.rest_world_pos.get(joint_name, om.MVector(0, 0, 0))
