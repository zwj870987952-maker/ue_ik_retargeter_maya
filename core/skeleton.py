# -*- coding: utf-8 -*-
"""
Skeleton representation and transform caching for Maya animation retargeting.
Parallels UE5's FRetargetSkeleton.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import get_world_transform, set_world_transform


class RetargetSkeleton(object):
    """
    Encapsulates a skeletal hierarchy in Maya.
    Caches reference poses, parent indices, and facilitates batch transformations.
    """

    def __init__(self, root_joint):
        self.root_joint = root_joint
        self.joint_names = []
        self.joint_indices = {}
        self.parent_indices = []
        self.rest_world_pos = {}
        self.rest_world_rot = {}
        self.rest_local_rot = {}

        self._build_hierarchy()
        self.cache_reference_tpose()

    def _build_hierarchy(self):
        """Discovers all joints under root_joint and builds indexing hierarchy."""
        all_joints = cmds.listRelatives(self.root_joint, ad=True, type='joint', f=False) or []
        all_joints.reverse()
        self.joint_names = [self.root_joint] + all_joints

        for idx, j_name in enumerate(self.joint_names):
            self.joint_indices[j_name] = idx

        self.parent_indices = [-1] * len(self.joint_names)
        for idx, j_name in enumerate(self.joint_names):
            parent = cmds.listRelatives(j_name, p=True, type='joint')
            if parent and parent[0] in self.joint_indices:
                self.parent_indices[idx] = self.joint_indices[parent[0]]
            else:
                self.parent_indices[idx] = -1

    def cache_reference_tpose(self):
        """
        Captures the pristine zero-rotation reference pose (T-pose) by temporarily
        disconnecting driving anim curves, caching rest transforms, and reconnecting.
        """
        connections = []
        for j_name in self.joint_names:
            for attr in ['rotateX', 'rotateY', 'rotateZ']:
                plug = '{}.{}'.format(j_name, attr)
                inputs = cmds.listConnections(plug, source=True, destination=False, plugs=True) or []
                if inputs:
                    cmds.disconnectAttr(inputs[0], plug)
                    connections.append((inputs[0], plug))
                cmds.setAttr(plug, 0.0)

        # Cache the pristine rest pose
        self.cache_rest_pose()

        # Restore animation connections
        for src, dst in connections:
            cmds.connectAttr(src, dst)

    def cache_rest_pose(self):
        """
        Records the current pose as the base reference/rest pose.
        Used for computing rotation deltas and length ratios.
        """
        for j_name in self.joint_names:
            pos, rot = get_world_transform(j_name)
            self.rest_world_pos[j_name] = pos
            self.rest_world_rot[j_name] = rot

            # Also cache local rotation
            rot_deg = cmds.getAttr('{}.rotate'.format(j_name))[0]
            from .math_utils import euler_deg_to_quat
            self.rest_local_rot[j_name] = euler_deg_to_quat(rot_deg[0], rot_deg[1], rot_deg[2])

    def get_bone_index(self, joint_name):
        return self.joint_indices.get(joint_name, -1)

    def get_parent_joint(self, joint_name):
        idx = self.get_bone_index(joint_name)
        if idx >= 0:
            p_idx = self.parent_indices[idx]
            if p_idx >= 0:
                return self.joint_names[p_idx]
        return None

    def get_current_pose(self):
        """Returns dict of current world positions and rotations for all joints."""
        pose = {}
        for j_name in self.joint_names:
            pos, rot = get_world_transform(j_name)
            pose[j_name] = (pos, rot)
        return pose

    def apply_pose(self, pose_dict):
        """Applies a dict of joint_name: (pos, rot) to Maya joints."""
        for j_name, (pos, rot) in pose_dict.items():
            if j_name in self.joint_indices:
                set_world_transform(j_name, pos, rot)
