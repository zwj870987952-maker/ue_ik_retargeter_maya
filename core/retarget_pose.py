# -*- coding: utf-8 -*-
"""
Retarget Base Pose alignment (e.g. A-Pose to T-Pose matching).
Parallels UE 5.8's FIKRetargetPose and FResolvedRetargetPose.
Properly propagates alignment rotations down the full skeletal hierarchy
using Maya OpenMaya 2.0's row-vector (left-to-right) MQuaternion convention.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import get_world_transform, set_world_transform, euler_deg_to_quat, rotate_vector_by_quat


class RetargetPoseManager(object):
    """
    Manages retarget pose alignment offsets without modifying skin bind poses.
    Stores both world-space alignment deltas (self.delta_rotations) and
    bone-local rotation offsets (self.local_delta_rotations, matching UE 5.8's
    FIKRetargetPose::BoneRotationOffsets).
    Propagates parent rotations to all descendant bones in the hierarchy,
    matching UE 5.8's FRetargetSkeleton::UpdateGlobalTransformsBelowBone.
    """

    def __init__(self, skeleton, pose_name="DefaultRetargetPose"):
        self.skeleton = skeleton
        self.pose_name = pose_name
        self.delta_rotations = {}        # joint_dag -> MQuaternion (world-space delta applied at bone)
        self.local_delta_rotations = {}  # joint_dag -> MQuaternion (bone-local delta matching UE 5.8 BoneRotationOffsets)
        self.root_translation_offset = om.MVector(0.0, 0.0, 0.0)
        self.aligned_world_pos = {}
        self.aligned_world_rot = {}
        self.aligned_local_pos = {}
        self.aligned_local_rot = {}
        self. is_editing = False
        self._temp_pose_cache = {}

        self.recompute_aligned_transforms()

    def recompute_aligned_transforms(self):
        """
        Recomputes aligned world and local transforms for all joints in topological order.
        Note on Maya OpenMaya MQuaternion convention:
        v.rotateBy(q1 * q2) == v.rotateBy(q1).rotateBy(q2) (left-to-right).
        Therefore:
        - local_rot = rest_rot * p_rest_rot.inverse()
        - propagated_rot = local_rot * p_aligned_rot
        - aligned_rot = propagated_rot * world_delta
        - aligned_local_rot = aligned_rot * p_aligned_rot.inverse()
        - local_delta = local_rot.inverse() * aligned_local_rot
        """
        self.aligned_world_pos = {}
        self.aligned_world_rot = {}
        self.aligned_local_pos = {}
        self.aligned_local_rot = {}
        self.local_delta_rotations = {}

        for idx, j_name in enumerate(self.skeleton.joint_names):
            p_idx = self.skeleton.parent_indices[idx]
            rest_pos = self.skeleton.rest_world_pos[j_name]
            rest_rot = self.skeleton.rest_world_rot[j_name]

            if p_idx == -1:
                # Root joint
                aligned_pos = rest_pos + self.root_translation_offset
                world_delta = self.delta_rotations.get(j_name, om.MQuaternion.kIdentity)
                aligned_rot = (rest_rot * world_delta).normal()
                local_delta = (rest_rot.inverse() * aligned_rot).normal()
                self.aligned_local_pos[j_name] = om.MVector(aligned_pos)
                self.aligned_local_rot[j_name] = om.MQuaternion(aligned_rot)
            else:
                p_name = self.skeleton.joint_names[p_idx]
                p_rest_pos = self.skeleton.rest_world_pos[p_name]
                p_rest_rot = self.skeleton.rest_world_rot[p_name]

                p_aligned_pos = self.aligned_world_pos[p_name]
                p_aligned_rot = self.aligned_world_rot[p_name]

                # Child rest transform relative to parent rest transform
                local_vec = rotate_vector_by_quat(rest_pos - p_rest_pos, p_rest_rot.inverse())
                local_rot = (rest_rot * p_rest_rot.inverse()).normal()

                # Propagated world position & rotation
                aligned_pos = p_aligned_pos + rotate_vector_by_quat(local_vec, p_aligned_rot)
                propagated_rot = (local_rot * p_aligned_rot).normal()

                # Apply explicit world alignment delta for this joint (if any)
                world_delta = self.delta_rotations.get(j_name, om.MQuaternion.kIdentity)
                aligned_rot = (propagated_rot * world_delta).normal()

                # Compute UE 5.8 equivalent local delta offset:
                # aligned_local_rot = local_rot * local_delta
                aligned_local_rot = (aligned_rot * p_aligned_rot.inverse()).normal()
                local_delta = (local_rot.inverse() * aligned_local_rot).normal()

                self.aligned_local_pos[j_name] = local_vec
                self.aligned_local_rot[j_name] = aligned_local_rot

            self.aligned_world_pos[j_name] = aligned_pos
            self.aligned_world_rot[j_name] = aligned_rot

            if not self._is_identity_quat(local_delta):
                self.local_delta_rotations[j_name] = local_delta

    @staticmethod
    def _is_identity_quat(q, tol=1e-5):
        return abs(q.x) < tol and abs(q.y) < tol and abs(q.z) < tol and abs(abs(q.w) - 1.0) < tol

    def set_world_delta_for_bone(self, joint_name, world_delta_quat, recompute=True):
        """Sets or accumulates a world-space alignment rotation delta on a joint."""
        dag = self.skeleton.resolve_bone_dag(joint_name) or joint_name
        if self._is_identity_quat(world_delta_quat):
            self.delta_rotations.pop(dag, None)
        else:
            self.delta_rotations[dag] = world_delta_quat.normal()
        if recompute:
            self.recompute_aligned_transforms()

    def apply_aligned_pose_to_scene(self):
        """
        Applies the computed aligned retarget pose directly to the Maya viewport skeleton
        in topological parent-to-child order so the user can visually inspect the pose.
        """
        for idx, j_name in enumerate(self.skeleton.joint_names):
            pos = self.aligned_world_pos.get(j_name, self.skeleton.rest_world_pos[j_name])
            rot = self.aligned_world_rot.get(j_name, self.skeleton.rest_world_rot[j_name])
            if idx == 0:
                set_world_transform(j_name, pos, rot)
            else:
                set_world_transform(j_name, quat=rot)

    def start_edit_mode(self):
        """Puts skeleton in interactive edit mode in the Maya viewport showing the aligned pose."""
        self.is_editing = True
        self._temp_pose_cache.clear()
        for j in self.skeleton.joint_names:
            pos, rot = get_world_transform(j)
            self._temp_pose_cache[j] = (pos, rot)
        self.apply_aligned_pose_to_scene()

    def save_edit_mode(self):
        """
        Saves current viewport joint orientations as the retarget alignment pose.
        Processes joints topologically so only true local edits relative to parent
        are recorded as deltas.
        """
        self.delta_rotations.clear()

        # Root translation delta
        root_name = self.skeleton.root_joint
        curr_root_pos, _ = get_world_transform(root_name)
        rest_root_pos = self.skeleton.rest_world_pos[root_name]
        self.root_translation_offset = curr_root_pos - rest_root_pos

        curr_world_rots = {}
        for j in self.skeleton.joint_names:
            _, curr_rot = get_world_transform(j)
            curr_world_rots[j] = curr_rot.normal()

        for idx, j_name in enumerate(self.skeleton.joint_names):
            p_idx = self.skeleton.parent_indices[idx]
            rest_rot = self.skeleton.rest_world_rot[j_name]
            curr_rot = curr_world_rots[j_name]

            if p_idx == -1:
                propagated_rot = rest_rot
            else:
                p_name = self.skeleton.joint_names[p_idx]
                p_rest_rot = self.skeleton.rest_world_rot[p_name]
                p_curr_rot = curr_world_rots[p_name]
                local_rot = (rest_rot * p_rest_rot.inverse()).normal()
                propagated_rot = (local_rot * p_curr_rot).normal()

            world_delta = (propagated_rot.inverse() * curr_rot).normal()
            if not self._is_identity_quat(world_delta):
                self.delta_rotations[j_name] = world_delta

        self.recompute_aligned_transforms()
        self.is_editing = False

    def reset_to_bind_pose(self):
        """Resets all skeleton joints back to original rest/bind pose."""
        self.delta_rotations.clear()
        self.local_delta_rotations.clear()
        self.root_translation_offset = om.MVector(0.0, 0.0, 0.0)
        self.recompute_aligned_transforms()
        for idx, j in enumerate(self.skeleton.joint_names):
            if idx == 0:
                set_world_transform(j, self.skeleton.rest_world_pos[j], self.skeleton.rest_world_rot[j])
            else:
                set_world_transform(j, quat=self.skeleton.rest_world_rot[j])
        self.is_editing = False

    def get_aligned_rest_rotation(self, joint_name):
        """Returns the rest rotation adjusted by retarget pose alignment delta and hierarchy propagation."""
        dag = self.skeleton.resolve_bone_dag(joint_name) or joint_name
        if dag in self.aligned_world_rot:
            return self.aligned_world_rot[dag]
        return self.skeleton.rest_world_rot.get(dag, om.MQuaternion.kIdentity)

    def get_aligned_rest_position(self, joint_name):
        """Returns rest position with retarget pose root offset and hierarchy propagation applied."""
        dag = self.skeleton.resolve_bone_dag(joint_name) or joint_name
        if dag in self.aligned_world_pos:
            return self.aligned_world_pos[dag]
        return self.skeleton.rest_world_pos.get(dag, om.MVector(0.0, 0.0, 0.0))
