# -*- coding: utf-8 -*-
"""
Rig driver module allowing retargeted results to drive Control Rig controllers directly.
Enables animators to perform secondary keyframing immediately after retargeting.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from ..core.math_utils import get_world_transform, set_world_transform


class ControlRigDriver(object):
    """
    Maps retargeted bone transforms to animation rig controls (IK/FK handles).
    """

    def __init__(self, processor):
        self.processor = processor
        self.control_mappings = {}  # {joint_name: control_name}

    def map_control(self, joint_name, control_name):
        """Pairs a target joint with an animation rig control."""
        if cmds.objExists(joint_name) and cmds.objExists(control_name):
            self.control_mappings[joint_name] = control_name

    def auto_detect_controls(self, control_set_or_grp=None):
        """Auto-discovers rig controls matching target joint names."""
        for j in self.processor.target_skeleton.joint_names:
            base_name = j.split(':')[-1]
            candidates = [
                'ctrl_' + base_name,
                base_name + '_ctrl',
                base_name + '_FK_ctrl',
                base_name + '_IK_ctrl',
                'FK_' + base_name,
                'IK_' + base_name
            ]
            for c in candidates:
                if cmds.objExists(c):
                    self.control_mappings[j] = c
                    break

    def apply_to_controls(self):
        """Transfers current target joint transforms to mapped rig controls."""
        for j, ctrl in self.control_mappings.items():
            pos, rot = get_world_transform(j)
            set_world_transform(ctrl, pos, rot)

    def bake_to_controls(self, start_frame, end_frame, step=1):
        """Bakes retargeted results directly onto control rig objects."""
        controls = list(self.control_mappings.values())
        if not controls:
            return

        orig_time = cmds.currentTime(q=True)
        try:
            for frame in range(int(start_frame), int(end_frame) + 1, int(step)):
                cmds.currentTime(frame, e=True)
                self.processor.retarget_frame()
                self.apply_to_controls()
                for ctrl in controls:
                    cmds.setKeyframe(ctrl, time=frame)
        finally:
            cmds.currentTime(orig_time, e=True)
