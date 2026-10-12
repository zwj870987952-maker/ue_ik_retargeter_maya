# -*- coding: utf-8 -*-
"""
Bone chains and chain mapping for animation retargeting.
Parallels UE5's FBoneChain and FRetargetChainSettings.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import slerp, lerp_vector


class BoneChain(object):
    """
    Represents an ordered chain of joints from start_bone to end_bone.
    Parameterizes the chain by arc length to support retargeting between
    skeletons with different bone counts (e.g. 3-bone spine to 4-bone spine).
    """

    def __init__(self, name, start_bone, end_bone, skeleton):
        self.name = name
        self.start_bone = start_bone
        self.end_bone = end_bone
        self.skeleton = skeleton
        self.bones = []
        self.params = []  # Param value in [0.0, 1.0] for each joint
        self.initial_lengths = []
        self.total_initial_length = 0.0

        self._build_chain()

    def _build_chain(self):
        """Discovers linear joint path between start_bone and end_bone."""
        if not cmds.objExists(self.start_bone) or not cmds.objExists(self.end_bone):
            return

        # Find path from end_bone up to start_bone
        chain_reversed = [self.end_bone]
        curr = self.end_bone

        while curr != self.start_bone:
            parent = self.skeleton.get_parent_joint(curr)
            if not parent:
                break
            chain_reversed.append(parent)
            curr = parent
            if curr == self.start_bone:
                break

        if curr != self.start_bone:
            # Fallback if hierarchy was not strictly linear: single joint chain
            self.bones = [self.start_bone]
            if self.end_bone != self.start_bone:
                self.bones.append(self.end_bone)
        else:
            self.bones = list(reversed(chain_reversed))

        # Calculate initial joint-to-joint lengths
        self.initial_lengths = [0.0]
        cumulative = 0.0
        for i in range(1, len(self.bones)):
            p0 = self.skeleton.rest_world_pos[self.bones[i - 1]]
            p1 = self.skeleton.rest_world_pos[self.bones[i]]
            length = (p1 - p0).length()
            cumulative += length
            self.initial_lengths.append(cumulative)

        self.total_initial_length = max(1e-4, cumulative)

        # Normalize parameters to [0.0, 1.0]
        self.params = [l / self.total_initial_length for l in self.initial_lengths]

    def get_bone_closest_to_param(self, u):
        """Finds joint index closest to parameter u in [0.0, 1.0]."""
        best_idx = 0
        min_diff = 999.0
        for idx, param in enumerate(self.params):
            diff = abs(param - u)
            if diff < min_diff:
                min_diff = diff
                best_idx = idx
        return best_idx

    def get_interpolated_transform_at_param(self, u, current_pose_dict, rest_pose=False):
        """
        Samples interpolated position and rotation along the chain at parameter u.
        Matches UE5's FResolvedBoneChain::GetTransformAtChainParam().
        """
        if len(self.bones) == 1:
            b = self.bones[0]
            if rest_pose:
                return self.skeleton.rest_world_pos[b], self.skeleton.rest_world_rot[b]
            return current_pose_dict[b]

        # Find segment [i, i+1] containing u
        seg_idx = 0
        for i in range(len(self.params) - 1):
            if self.params[i] <= u <= self.params[i + 1]:
                seg_idx = i
                break
            elif u > self.params[-1]:
                seg_idx = len(self.params) - 2

        u0 = self.params[seg_idx]
        u1 = self.params[seg_idx + 1]
        denom = max(1e-4, u1 - u0)
        local_alpha = (u - u0) / denom
        local_alpha = max(0.0, min(1.0, local_alpha))

        b0 = self.bones[seg_idx]
        b1 = self.bones[seg_idx + 1]

        if rest_pose:
            p0, r0 = self.skeleton.rest_world_pos[b0], self.skeleton.rest_world_rot[b0]
            p1, r1 = self.skeleton.rest_world_pos[b1], self.skeleton.rest_world_rot[b1]
        else:
            p0, r0 = current_pose_dict[b0]
            p1, r1 = current_pose_dict[b1]

        interp_pos = lerp_vector(p0, p1, local_alpha)
        interp_rot = slerp(r0, r1, local_alpha)
        return interp_pos, interp_rot


class ChainMapping(object):
    """
    Pairs a source bone chain with a target bone chain and defines solver behaviors.
    """

    def __init__(self, name, source_chain, target_chain, enable_ik=False, ik_goal_name=None):
        self.name = name
        self.source_chain = source_chain
        self.target_chain = target_chain
        self.enable_ik = enable_ik
        self.ik_goal_name = ik_goal_name or name + "_IKGoal"
        self.rotation_mode = "Interpolated"  # "Interpolated", "OneToOne", "CopyLocal"
        self.translation_mode = "None"      # "None", "GloballyScaled", "StretchBoneLength"


# Pre-defined templates for automatic chain detection
CHAIN_PATTERNS = {
    'Pelvis': {
        'patterns': ['*pelvis*', '*hips*', '*hip*', '*root_joint*']
    },
    'Spine': {
        'start': ['*spine*', '*spine1*', '*spine_01*'],
        'end': ['*chest*', '*spine3*', '*spine_03*', '*spine2*']
    },
    'Head': {
        'start': ['*neck*', '*neck_01*'],
        'end': ['*head*']
    },
    'LeftArm': {
        'start': ['*upperarm_l*', '*leftarm*', '*l_arm*', '*arm_l*', '*l_upperarm*'],
        'end': ['*hand_l*', '*lefthand*', '*l_hand*'],
        'is_limb': True
    },
    'RightArm': {
        'start': ['*upperarm_r*', '*rightarm*', '*r_arm*', '*arm_r*', '*r_upperarm*'],
        'end': ['*hand_r*', '*righthand*', '*r_hand*'],
        'is_limb': True
    },
    'LeftLeg': {
        'start': ['*thigh_l*', '*leftupleg*', '*l_upleg*', '*leftleg*', '*l_thigh*'],
        'end': ['*foot_l*', '*leftfoot*', '*l_foot*'],
        'is_limb': True
    },
    'RightLeg': {
        'start': ['*thigh_r*', '*rightupleg*', '*r_upleg*', '*rightleg*', '*r_thigh*'],
        'end': ['*foot_r*', '*rightfoot*', '*r_foot*'],
        'is_limb': True
    }
}
