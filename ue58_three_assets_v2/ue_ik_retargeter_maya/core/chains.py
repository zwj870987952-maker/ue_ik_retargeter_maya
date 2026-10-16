# -*- coding: utf-8 -*-
"""
Bone chains, template characterization, and chain mapping for animation retargeting.
Parallels UE 5.8's FBoneChain, FResolvedBoneChain (IKRetargetProcessor.cpp),
and FKnownTemplateHierarchies / FAutoCharacterizer (IKRigAutoCharacterizer.cpp).
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import slerp, lerp_vector


class BoneChain(object):
    """
    Represents an ordered chain of joints from start_bone to end_bone.
    Stores full Maya DAG paths in self.bones so duplicate joint names across
    two skeletons in the same scene never collide.
    """

    def __init__(self, name, start_bone, end_bone, skeleton, ik_goal_name=""):
        self.name = name
        self.skeleton = skeleton
        self.start_bone = skeleton.resolve_bone_dag(start_bone) or start_bone
        self.end_bone = skeleton.resolve_bone_dag(end_bone) or end_bone
        self.ik_goal_name = ik_goal_name or ""

        self.bones = []             # Full DAG paths from start_bone to end_bone
        self.clean_bones = []       # Stripped clean bone names
        self.params = []            # Normalized parameter in [0.0, 1.0] for each joint
        self.initial_lengths = []
        self.total_initial_length = 0.0

        self._build_chain()

    def _build_chain(self, custom_pos_dict=None):
        """
        Discovers linear joint path between start_bone and end_bone,
        and calculates normalized arc-length parameters (UE 5.8 CalculateBoneParameters).
        """
        if not cmds.objExists(self.start_bone) or not cmds.objExists(self.end_bone):
            return

        if self.start_bone == self.end_bone:
            self.bones = [self.start_bone]
        else:
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
                self.bones = [self.start_bone, self.end_bone]
            else:
                self.bones = list(reversed(chain_reversed))

        self.clean_bones = [self.skeleton.get_clean_name(b) for b in self.bones]

        pos_map = custom_pos_dict if custom_pos_dict is not None else self.skeleton.rest_world_pos

        # Single-bone chain special case (UE 5.8 IKRetargetProcessor.cpp line 904: Params.Add(1.0f))
        if len(self.bones) == 1:
            self.initial_lengths = [0.0]
            self.total_initial_length = 0.0
            self.params = [1.0]
            return

        self.initial_lengths = [0.0]
        cumulative = 0.0
        for i in range(1, len(self.bones)):
            p0 = pos_map[self.bones[i - 1]]
            p1 = pos_map[self.bones[i]]
            length = (p1 - p0).length()
            cumulative += length
            self.initial_lengths.append(cumulative)

        self.total_initial_length = max(1e-4, cumulative)
        self.params = [l / self.total_initial_length for l in self.initial_lengths]

    def get_param_of_bone(self, bone_dag_or_name):
        """Returns the normalized chain parameter [0.0, 1.0] of a bone in this chain."""
        dag = self.skeleton.resolve_bone_dag(bone_dag_or_name)
        if dag in self.bones:
            idx = self.bones.index(dag)
            return self.params[idx]
        return -1.0

    def get_bone_closest_to_param(self, u):
        """Finds joint index in self.bones closest to parameter u in [0.0, 1.0]."""
        best_idx = 0
        min_diff = 999.0
        for idx, param in enumerate(self.params):
            diff = abs(param - u)
            if diff < min_diff:
                min_diff = diff
                best_idx = idx
        return best_idx

    def get_interpolated_transform_at_param(self, u, current_pose_dict=None, rest_pose=False, pose_mgr=None):
        """
        Samples interpolated position and rotation along the chain at parameter u.
        Matches UE 5.8's FResolvedBoneChain::GetTransformAtChainParam().
        """
        if not self.bones:
            return om.MVector(0, 0, 0), om.MQuaternion.kIdentity

        def _get_bone_xform(b):
            if pose_mgr is not None:
                return pose_mgr.get_aligned_rest_position(b), pose_mgr.get_aligned_rest_rotation(b)
            if rest_pose or current_pose_dict is None:
                return self.skeleton.rest_world_pos[b], self.skeleton.rest_world_rot[b]
            return current_pose_dict[b]

        if len(self.bones) == 1 or u <= 1e-5:
            return _get_bone_xform(self.bones[0])

        if u >= 1.0 - 1e-5:
            return _get_bone_xform(self.bones[-1])

        seg_idx = len(self.params) - 2
        for i in range(1, len(self.params)):
            if self.params[i] > u:
                seg_idx = i - 1
                break

        u0 = self.params[seg_idx]
        u1 = self.params[seg_idx + 1]
        denom = max(1e-5, u1 - u0)
        local_alpha = max(0.0, min(1.0, (u - u0) / denom))

        b0 = self.bones[seg_idx]
        b1 = self.bones[seg_idx + 1]
        p0, r0 = _get_bone_xform(b0)
        p1, r1 = _get_bone_xform(b1)

        interp_pos = lerp_vector(p0, p1, local_alpha)
        from .math_utils import fast_lerp
        interp_rot = fast_lerp(r0, r1, local_alpha)
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
        self.ik_goal_name = ik_goal_name or target_chain.ik_goal_name or (name + "_IKGoal" if enable_ik else "")
        self.enable_fk = True
        self.rotation_mode = "Interpolated"  # "Interpolated", "OneToOne", "CopyLocal"
        self.rotation_alpha = 1.0
        self.translation_mode = "None"      # "None", "GloballyScaled", "StretchBoneLength"
        self.translation_alpha = 1.0


# ==============================================================================
# UE 5.8 Standard Known Templates (Ported from IKRigAutoCharacterizer.cpp)
# ==============================================================================
KNOWN_SKELETON_TEMPLATES = {
    "Mixamo": {
        "root_bone": "",
        "pelvis_bone": "Hips",
        "bones_to_exclude_from_auto_pose": ["LeftFoot", "RightFoot"],
        "chains": [
            ("Spine",         "Spine",           "Spine2",          ""),
            ("Neck",          "Neck",            "Neck",            ""),
            ("Head",          "Head",            "Head",            ""),
            ("LeftClavicle",  "LeftShoulder",    "LeftShoulder",    ""),
            ("LeftArm",       "LeftArm",         "LeftHand",        "LeftHandIK"),
            ("LeftThumb",     "LeftHandThumb1",  "LeftHandThumb3",  ""),
            ("LeftIndex",     "LeftHandIndex1",  "LeftHandIndex3",  ""),
            ("LeftMiddle",    "LeftHandMiddle1", "LeftHandMiddle3", ""),
            ("LeftRing",      "LeftHandRing1",   "LeftHandRing3",   ""),
            ("LeftPinky",     "LeftHandPinky1",  "LeftHandPinky3",  ""),
            ("RightClavicle", "RightShoulder",   "RightShoulder",   ""),
            ("RightArm",      "RightArm",        "RightHand",       "RightHandIK"),
            ("RightThumb",    "RightHandThumb1", "RightHandThumb3", ""),
            ("RightIndex",    "RightHandIndex1", "RightHandIndex3", ""),
            ("RightMiddle",   "RightHandMiddle1","RightHandMiddle3",""),
            ("RightRing",     "RightHandRing1",  "RightHandRing3",  ""),
            ("RightPinky",    "RightHandPinky1", "RightHandPinky3", ""),
            ("LeftLeg",       "LeftUpLeg",       "LeftFoot",        "LeftFootIK"),
            ("LeftFoot",      "LeftToeBase",     "LeftToeBase",     ""),
            ("RightLeg",      "RightUpLeg",      "RightFoot",       "RightFootIK"),
            ("RightFoot",     "RightToeBase",    "RightToeBase",    ""),
        ],
        "fbik_bone_settings": {
            "Hips":          {"rotation_stiffness": 0.95},
            "LeftShoulder":  {"rotation_stiffness": 0.95},
            "RightShoulder": {"rotation_stiffness": 0.95},
            "LeftFoot":      {"rotation_stiffness": 0.85},
            "RightFoot":     {"rotation_stiffness": 0.85},
            "LeftLeg":       {"preferred_axis": "+X", "is_hinge": True},
            "RightLeg":      {"preferred_axis": "+X", "is_hinge": True},
            "LeftForeArm":   {"preferred_axis": "+Y", "is_hinge": True},
            "RightForeArm":  {"preferred_axis": "-Y", "is_hinge": True},
        },
        "bones_to_pin": [],
    },
    "UE5_Mannequin": {
        "root_bone": "root",
        "pelvis_bone": "pelvis",
        "bones_to_exclude_from_auto_pose": ["foot_l", "foot_r"],
        "chains": [
            ("Spine",                 "spine_01",            "spine_05",            ""),
            ("Neck",                  "neck_01",             "neck_02",             ""),
            ("Head",                  "head",                "head",                ""),
            ("LeftClavicle",          "clavicle_l",          "clavicle_l",          ""),
            ("LeftArm",               "upperarm_l",          "hand_l",              "LeftHandIK"),
            ("LeftIndexMetacarpal",   "index_metacarpal_l",  "index_metacarpal_l",  ""),
            ("LeftMiddleMetacarpal",  "middle_metacarpal_l", "middle_metacarpal_l", ""),
            ("LeftRingMetacarpal",    "ring_metacarpal_l",   "ring_metacarpal_l",   ""),
            ("LeftPinkyMetacarpal",   "pinky_metacarpal_l",  "pinky_metacarpal_l",  ""),
            ("LeftThumb",             "thumb_01_l",          "thumb_03_l",          ""),
            ("LeftIndex",             "index_01_l",          "index_03_l",          ""),
            ("LeftMiddle",            "middle_01_l",         "middle_03_l",         ""),
            ("LeftRing",              "ring_01_l",           "ring_03_l",           ""),
            ("LeftPinky",             "pinky_01_l",          "pinky_03_l",          ""),
            ("RightClavicle",         "clavicle_r",          "clavicle_r",          ""),
            ("RightArm",              "upperarm_r",          "hand_r",              "RightHandIK"),
            ("RightIndexMetacarpal",  "index_metacarpal_r",  "index_metacarpal_r",  ""),
            ("RightMiddleMetacarpal", "middle_metacarpal_r", "middle_metacarpal_r", ""),
            ("RightRingMetacarpal",   "ring_metacarpal_r",   "ring_metacarpal_r",   ""),
            ("RightPinkyMetacarpal",  "pinky_metacarpal_r",  "pinky_metacarpal_r",  ""),
            ("RightThumb",            "thumb_01_r",          "thumb_03_r",          ""),
            ("RightIndex",            "index_01_r",          "index_03_r",          ""),
            ("RightMiddle",           "middle_01_r",         "middle_03_r",         ""),
            ("RightRing",             "ring_01_r",           "ring_03_r",           ""),
            ("RightPinky",            "pinky_01_r",          "pinky_03_r",          ""),
            ("LeftLeg",               "thigh_l",             "foot_l",              "LeftFootIK"),
            ("LeftFoot",              "ball_l",              "ball_l",              ""),
            ("RightLeg",              "thigh_r",             "foot_r",              "RightFootIK"),
            ("RightFoot",             "ball_r",              "ball_r",              ""),
        ],
        "fbik_bone_settings": {
            "pelvis":     {"rotation_stiffness": 0.95},
            "clavicle_l": {"rotation_stiffness": 0.90},
            "clavicle_r": {"rotation_stiffness": 0.90},
            "foot_l":     {"rotation_stiffness": 0.85},
            "foot_r":     {"rotation_stiffness": 0.85},
            "calf_l":     {"preferred_axis": "+Z", "is_hinge": False},
            "calf_r":     {"preferred_axis": "+Z", "is_hinge": False},
            "lowerarm_l": {"preferred_axis": "+Z", "is_hinge": False},
            "lowerarm_r": {"preferred_axis": "+Z", "is_hinge": False},
            "spine_05":   {"excluded": True},
        },
        "bones_to_pin": [
            ("ik_foot_root", "root"),
            ("ik_foot_l",    "foot_l"),
            ("ik_foot_r",    "foot_r"),
            ("ik_hand_root", "root"),
            ("ik_hand_gun",  "hand_r"),
            ("ik_hand_l",    "hand_l"),
            ("ik_hand_r",    "hand_r"),
        ],
    },
    "UE4_Mannequin": {
        "root_bone": "root",
        "pelvis_bone": "pelvis",
        "bones_to_exclude_from_auto_pose": ["foot_l", "foot_r"],
        "chains": [
            ("Spine",         "spine_01",   "spine_03",   ""),
            ("Neck",          "neck_01",    "neck_01",    ""),
            ("Head",          "head",       "head",       ""),
            ("LeftClavicle",  "clavicle_l", "clavicle_l", ""),
            ("LeftArm",       "upperarm_l", "hand_l",     "LeftHandIK"),
            ("LeftThumb",     "thumb_01_l", "thumb_03_l", ""),
            ("LeftIndex",     "index_01_l", "index_03_l", ""),
            ("LeftMiddle",    "middle_01_l","middle_03_l",""),
            ("LeftRing",      "ring_01_l",  "ring_03_l",  ""),
            ("LeftPinky",     "pinky_01_l", "pinky_03_l", ""),
            ("RightClavicle", "clavicle_r", "clavicle_r", ""),
            ("RightArm",      "upperarm_r", "hand_r",     "RightHandIK"),
            ("RightThumb",    "thumb_01_r", "thumb_03_r", ""),
            ("RightIndex",    "index_01_r", "index_03_r", ""),
            ("RightMiddle",   "middle_01_r","middle_03_r",""),
            ("RightRing",     "ring_01_r",  "ring_03_r",  ""),
            ("RightPinky",    "pinky_01_r", "pinky_03_r", ""),
            ("LeftLeg",       "thigh_l",    "foot_l",     "LeftFootIK"),
            ("LeftFoot",      "ball_l",     "ball_l",     ""),
            ("RightLeg",      "thigh_r",    "foot_r",     "RightFootIK"),
            ("RightFoot",     "ball_r",     "ball_r",     ""),
        ],
        "fbik_bone_settings": {
            "pelvis":     {"rotation_stiffness": 0.95},
            "clavicle_l": {"rotation_stiffness": 0.90},
            "clavicle_r": {"rotation_stiffness": 0.90},
            "foot_l":     {"rotation_stiffness": 0.85},
            "foot_r":     {"rotation_stiffness": 0.85},
            "calf_l":     {"preferred_axis": "+Z", "is_hinge": False},
            "calf_r":     {"preferred_axis": "+Z", "is_hinge": False},
            "lowerarm_l": {"preferred_axis": "+Z", "is_hinge": False},
            "lowerarm_r": {"preferred_axis": "+Z", "is_hinge": False},
            "spine_03":   {"excluded": True},
        },
        "bones_to_pin": [
            ("ik_foot_root", "root"),
            ("ik_foot_l",    "foot_l"),
            ("ik_foot_r",    "foot_r"),
            ("ik_hand_root", "root"),
            ("ik_hand_gun",  "hand_r"),
            ("ik_hand_l",    "hand_l"),
            ("ik_hand_r",    "hand_r"),
        ],
    },
    "Reallusion_CC4": {
        "root_bone": "BoneRoot",
        "pelvis_bone": "Hip",
        "bones_to_exclude_from_auto_pose": ["L_Foot", "R_Foot"],
        "chains": [
            ("Spine",         "Waist",       "Spine02",     ""),
            ("Neck",          "NeckTwist01", "NeckTwist02", ""),
            ("Head",          "Head",        "Head",        ""),
            ("LeftClavicle",  "L_Clavicle",  "L_Clavicle",  ""),
            ("LeftArm",       "L_Upperarm",  "L_Hand",      "LeftHandIK"),
            ("LeftThumb",     "L_Thumb1",    "L_Thumb3",    ""),
            ("LeftIndex",     "L_Index1",    "L_Index3",    ""),
            ("LeftMiddle",    "L_Mid1",      "L_Mid3",      ""),
            ("LeftRing",      "L_Ring1",     "L_Ring3",     ""),
            ("LeftPinky",     "L_Pinky1",    "L_Pinky3",    ""),
            ("RightClavicle", "R_Clavicle",  "R_Clavicle",  ""),
            ("RightArm",      "R_Upperarm",  "R_Hand",      "RightHandIK"),
            ("RightThumb",    "R_Thumb1",    "R_Thumb3",    ""),
            ("RightIndex",    "R_Index1",    "R_Index3",    ""),
            ("RightMiddle",   "R_Mid1",      "R_Mid3",      ""),
            ("RightRing",     "R_Ring1",     "R_Ring3",     ""),
            ("RightPinky",    "R_Pinky1",    "R_Pinky3",    ""),
            ("LeftLeg",       "L_Thigh",     "L_Foot",      "LeftFootIK"),
            ("LeftFoot",      "L_ToeBase",   "L_ToeBase",   ""),
            ("RightLeg",      "R_Thigh",     "R_Foot",      "RightFootIK"),
            ("RightFoot",     "R_ToeBase",   "R_ToeBase",   ""),
            ("LeftBigToe", "L_BigToe1", "L_BigToe1", ""),
            ("LeftIndexToe", "L_IndexToe1", "L_IndexToe1", ""),
            ("LeftMiddleToe", "L_MidToe1", "L_MidToe1", ""),
            ("LeftRingToe", "L_RingToe1", "L_RingToe1", ""),
            ("LeftPinkyToe", "L_PinkyToe1", "L_PinkyToe1", ""),
            ("RightBigToe", "R_BigToe1", "R_BigToe1", ""),
            ("RightIndexToe", "R_IndexToe1", "R_IndexToe1", ""),
            ("RightMiddleToe", "R_MidToe1", "R_MidToe1", ""),
            ("RightRingToe", "R_RingToe1", "R_RingToe1", ""),
            ("RightPinkyToe", "R_PinkyToe1", "R_PinkyToe1", ""),
        ],
        "fbik_bone_settings": {
            "Hip":        {"rotation_stiffness": 0.95},
            "L_Clavicle": {"rotation_stiffness": 0.95},
            "R_Clavicle": {"rotation_stiffness": 0.95},
            "L_Foot":     {"rotation_stiffness": 0.85},
            "R_Foot":     {"rotation_stiffness": 0.85},
            "L_Calf":     {"preferred_axis": "-X", "is_hinge": True},
            "R_Calf":     {"preferred_axis": "-X", "is_hinge": True},
            "L_Forearm":  {"preferred_axis": "+X", "is_hinge": True},
            "R_Forearm":  {"preferred_axis": "+X", "is_hinge": True},
            "Pelvis":     {"excluded": True},
        },
        "bones_to_pin": [],
    },
    "SMPL_Humanoid": {
        "root_bone": "",
        "pelvis_bone": "Pelvis",
        "bones_to_exclude_from_auto_pose": ["L_Ankle", "R_Ankle"],
        "chains": [
            ("Spine",         "Spine1",     "Spine3",    ""),
            ("Neck",          "Neck",       "Neck",      ""),
            ("Head",          "Head",       "Head",      ""),
            ("LeftClavicle",  "L_Collar",   "L_Collar",  ""),
            ("LeftArm",       "L_Shoulder", "L_Wrist",   "LeftHandIK"),
            ("RightClavicle", "R_Collar",   "R_Collar",  ""),
            ("RightArm",      "R_Shoulder", "R_Wrist",   "RightHandIK"),
            ("LeftLeg",       "L_Hip",      "L_Ankle",   "LeftFootIK"),
            ("LeftFoot",      "L_Foot",     "L_Foot",    ""),
            ("RightLeg",      "R_Hip",      "R_Ankle",   "RightFootIK"),
            ("RightFoot",     "R_Foot",     "R_Foot",    ""),
        ],
        "fbik_bone_settings": {
            "Pelvis":   {"rotation_stiffness": 0.95},
            "L_Collar": {"rotation_stiffness": 0.90},
            "R_Collar": {"rotation_stiffness": 0.90},
            "L_Ankle":  {"rotation_stiffness": 0.85},
            "R_Ankle":  {"rotation_stiffness": 0.85},
            "L_Knee":   {"preferred_axis": "+X", "is_hinge": False},
            "R_Knee":   {"preferred_axis": "+X", "is_hinge": False},
            "L_Elbow":  {"preferred_axis": "+Y", "is_hinge": False},
            "R_Elbow":  {"preferred_axis": "-Y", "is_hinge": False},
        },
        "bones_to_pin": [],
    },
}

# Fallback heuristic patterns for custom skeletons not in KNOWN_SKELETON_TEMPLATES
CHAIN_PATTERNS = {
    "Pelvis": {
        "patterns": ["pelvis", "hips", "hip", "spine_c0_pelvis_jnt", "root_x"]
    },
    "Spine": {
        "start": ["spine_01", "spine1", "spine", "waist", "abdomenlower", "spine_c0_01_jnt"],
        "end":   ["spine_05", "spine_04", "spine_03", "spine3", "spine2", "spine02", "chestupper", "chest", "spine_c0_05_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "Neck": {
        "start": ["neck_01", "necktwist01", "necklower", "neck", "neck_c0_01_jnt"],
        "end":   ["neck_02", "necktwist02", "neckupper", "neck_01", "neck", "neck_c0_02_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "Head": {
        "start": ["head", "neck_c0_head_jnt"],
        "end":   ["head", "neck_c0_head_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "LeftClavicle": {
        "start": ["clavicle_l", "leftshoulder", "l_clavicle", "lcollar", "l_shoulder", "shoulder_l0_0_jnt"],
        "end":   ["clavicle_l", "leftshoulder", "l_clavicle", "lcollar", "l_shoulder", "shoulder_l0_0_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "RightClavicle": {
        "start": ["clavicle_r", "rightshoulder", "r_clavicle", "rcollar", "r_shoulder", "shoulder_r0_0_jnt"],
        "end":   ["clavicle_r", "rightshoulder", "r_clavicle", "rcollar", "r_shoulder", "shoulder_r0_0_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "LeftArm": {
        "start": ["upperarm_l", "leftarm", "l_upperarm", "lshldrbend", "arm_l0_upperarm_jnt"],
        "end":   ["hand_l", "lefthand", "l_hand", "lhand", "arm_l0_hand_jnt"],
        "is_limb": False,
        "goal": "LeftHandIK",
    },
    "RightArm": {
        "start": ["upperarm_r", "rightarm", "r_upperarm", "rshldrbend", "arm_r0_upperarm_jnt"],
        "end":   ["hand_r", "righthand", "r_hand", "rhand", "arm_r0_hand_jnt"],
        "is_limb": False,
        "goal": "RightHandIK",
    },
    "LeftThumb": {
        "start": ["thumb_01_l", "lefthandthumb1", "l_thumb1", "lthumb1", "thumb_l0_0_jnt"],
        "end":   ["thumb_03_l", "lefthandthumb3", "l_thumb3", "lthumb3", "thumb_l0_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "LeftIndex": {
        "start": ["index_01_l", "lefthandindex1", "l_index1", "lindex1", "finger_l0_0_jnt"],
        "end":   ["index_03_l", "lefthandindex3", "l_index3", "lindex3", "finger_l0_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "LeftMiddle": {
        "start": ["middle_01_l", "lefthandmiddle1", "l_mid1", "lmid1", "finger_l1_0_jnt"],
        "end":   ["middle_03_l", "lefthandmiddle3", "l_mid3", "lmid3", "finger_l1_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "LeftRing": {
        "start": ["ring_01_l", "lefthandring1", "l_ring1", "lring1", "finger_l2_0_jnt"],
        "end":   ["ring_03_l", "lefthandring3", "l_ring3", "lring3", "finger_l2_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "LeftPinky": {
        "start": ["pinky_01_l", "lefthandpinky1", "l_pinky1", "lpinky1", "finger_l3_0_jnt"],
        "end":   ["pinky_03_l", "lefthandpinky3", "l_pinky3", "lpinky3", "finger_l3_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "RightThumb": {
        "start": ["thumb_01_r", "righthandthumb1", "r_thumb1", "rthumb1", "thumb_r0_0_jnt"],
        "end":   ["thumb_03_r", "righthandthumb3", "r_thumb3", "rthumb3", "thumb_r0_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "RightIndex": {
        "start": ["index_01_r", "righthandindex1", "r_index1", "rindex1", "finger_r0_0_jnt"],
        "end":   ["index_03_r", "righthandindex3", "r_index3", "rindex3", "finger_r0_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "RightMiddle": {
        "start": ["middle_01_r", "righthandmiddle1", "r_mid1", "rmid1", "finger_r1_0_jnt"],
        "end":   ["middle_03_r", "righthandmiddle3", "r_mid3", "rmid3", "finger_r1_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "RightRing": {
        "start": ["ring_01_r", "righthandring1", "r_ring1", "rring1", "finger_r2_0_jnt"],
        "end":   ["ring_03_r", "righthandring3", "r_ring3", "rring3", "finger_r2_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "RightPinky": {
        "start": ["pinky_01_r", "righthandpinky1", "r_pinky1", "rpinky1", "finger_r3_0_jnt"],
        "end":   ["pinky_03_r", "righthandpinky3", "r_pinky3", "rpinky3", "finger_r3_2_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "LeftLeg": {
        "start": ["thigh_l", "leftupleg", "l_thigh", "lthighbend", "l_upleg", "leg_l0_thigh_jnt"],
        "end":   ["foot_l", "leftfoot", "l_foot", "lfoot", "leg_l0_foot_jnt"],
        "is_limb": False,  # UE 5.8 Auto-Retarget disables IK pass by default; IK Goal is still created
        "goal": "LeftFootIK",
    },
    "LeftFoot": {
        "start": ["ball_l", "lefttoebase", "l_toebase", "ltoe", "foot_l0_ball_jnt"],
        "end":   ["ball_l", "lefttoebase", "l_toebase", "ltoe", "foot_l0_ball_jnt"],
        "is_limb": False,
        "goal": "",
    },
    "RightLeg": {
        "start": ["thigh_r", "rightupleg", "r_thigh", "rthighbend", "r_upleg", "leg_r0_thigh_jnt"],
        "end":   ["foot_r", "rightfoot", "r_foot", "rfoot", "leg_r0_foot_jnt"],
        "is_limb": False,
        "goal": "RightFootIK",
    },
    "RightFoot": {
        "start": ["ball_r", "righttoebase", "r_toebase", "rtoe", "foot_r0_ball_jnt"],
        "end":   ["ball_r", "righttoebase", "r_toebase", "rtoe", "foot_r0_ball_jnt"],
        "is_limb": False,
        "goal": "",
    },
}


class AutoCharacterizer(object):
    """
    Parallels UE 5.8's FAutoCharacterizer (IKRigAutoCharacterizer.cpp).
    Scores a RetargetSkeleton against known standard hierarchy templates
    and generates a complete UIKRigDefinition dictionary (Pelvis, Root, 21+ BoneChains,
    4 IK Effector Goals, FBIK bone settings, and BonesToExcludeFromAutoPose).
    """

    @staticmethod
    def characterize(skeleton):
        """
        Inspects skeleton and returns a characterization dict:
        {
            "template_name": str,
            "match_score": float,
            "root_bone_dag": str or None,
            "pelvis_bone_dag": str,
            "root_bone_clean": str,
            "pelvis_bone_clean": str,
            "bones_to_exclude_from_auto_pose": [clean_names],
            "chains": [BoneChain, ...],
            "goals": [dict, ...],
            "fbik_bone_settings": dict,
            "bones_to_pin": list,
        }
        """
        best_template = None
        best_score = 0.0
        core_chain_names = {
            "Spine", "Neck", "Head",
            "LeftClavicle", "RightClavicle",
            "LeftArm", "RightArm",
            "LeftLeg", "RightLeg",
            "LeftFoot", "RightFoot",
        }

        for tpl_name, tpl_def in KNOWN_SKELETON_TEMPLATES.items():
            req_bones = set()
            core_bones = set()
            if tpl_def["pelvis_bone"]:
                req_bones.add(tpl_def["pelvis_bone"].lower())
                core_bones.add(tpl_def["pelvis_bone"].lower())
            for c_n, s_b, e_b, _ in tpl_def["chains"]:
                req_bones.add(s_b.lower())
                req_bones.add(e_b.lower())
                if c_n in core_chain_names:
                    core_bones.add(s_b.lower())
                    core_bones.add(e_b.lower())

            matched_all = sum(1 for b in req_bones if b in skeleton.lower_clean_to_dag)
            matched_core = sum(1 for b in core_bones if b in skeleton.lower_clean_to_dag)
            full_score = float(matched_all) / max(1, len(req_bones))
            core_score = float(matched_core) / max(1, len(core_bones))
            score = 0.70 * core_score + 0.30 * full_score
            if score > best_score:
                best_score = score
                best_template = tpl_name

        if best_template and best_score >= 0.60:
            tpl = KNOWN_SKELETON_TEMPLATES[best_template]
            root_dag = skeleton.resolve_bone_dag(tpl["root_bone"]) if tpl["root_bone"] else None
            pelvis_dag = skeleton.resolve_bone_dag(tpl["pelvis_bone"]) or skeleton.root_joint
            excluded = list(tpl["bones_to_exclude_from_auto_pose"])

            chains = []
            for c_name, s_name, e_name, goal_name in tpl["chains"]:
                s_dag = skeleton.resolve_bone_dag(s_name)
                e_dag = skeleton.resolve_bone_dag(e_name)
                if s_dag and e_dag:
                    chains.append(BoneChain(c_name, s_dag, e_dag, skeleton, ik_goal_name=goal_name))

            fbik_settings = {name:dict(value) for name,value in tpl.get("fbik_bone_settings", {}).items()}
            bones_to_pin = list(tpl.get("bones_to_pin", []))
            template_used = best_template
        else:
            # Heuristic exact-token fallback
            template_used = "Custom_Biped_Heuristic"
            best_score = 0.5

            def _find_exact(candidates):
                for cand in candidates:
                    c_low = cand.lower().replace("*", "")
                    if c_low in skeleton.lower_clean_to_dag:
                        return skeleton.lower_clean_to_dag[c_low]
                return None

            pelvis_dag = _find_exact(CHAIN_PATTERNS["Pelvis"]["patterns"]) or skeleton.root_joint
            root_dag = skeleton.root_joint if skeleton.root_joint != pelvis_dag else None
            chains = []
            excluded = []
            for c_name, c_def in CHAIN_PATTERNS.items():
                if c_name == "Pelvis":
                    continue
                s_dag = _find_exact(c_def["start"])
                e_dag = _find_exact(c_def["end"])
                if s_dag and e_dag:
                    goal_name = c_def.get("goal", "")
                    chain_obj = BoneChain(c_name, s_dag, e_dag, skeleton, ik_goal_name=goal_name)
                    chains.append(chain_obj)
                    if c_name in ("LeftLeg", "RightLeg"):
                        excluded.append(skeleton.get_clean_name(e_dag))

            fbik_settings = {}
            bones_to_pin = []

        # Build chain descriptors, bone_chains dict, and the 4 IK Effector Goals
        bone_chains_dict = {}
        chain_descriptors = []
        goals = []
        for ch in chains:
            bone_chains_dict[ch.name] = ch
            chain_descriptors.append({
                "chain_name": ch.name,
                "start_bone": skeleton.get_clean_name(ch.start_bone),
                "end_bone": skeleton.get_clean_name(ch.end_bone),
                "start_bone_dag": ch.start_bone,
                "end_bone_dag": ch.end_bone,
                "ik_goal_name": ch.ik_goal_name or "",
            })
            if ch.ik_goal_name and ch.bones:
                end_dag = ch.bones[-1]
                pos = skeleton.rest_world_pos[end_dag]
                rot = skeleton.rest_world_rot[end_dag]
                goals.append({
                    "goal_name": ch.ik_goal_name,
                    "chain_name": ch.name,
                    "bone_dag": end_dag,
                    "bone_name": skeleton.get_clean_name(end_dag),
                    "bone_clean": skeleton.get_clean_name(end_dag),
                    "position_alpha": 1.0,
                    "rotation_alpha": 1.0,
                    "initial_pos": [float(pos.x), float(pos.y), float(pos.z)],
                    "initial_rot": [float(rot.x), float(rot.y), float(rot.z), float(rot.w)],
                })

        from .pole_vector import calibrate_knee_settings
        fbik_settings = calibrate_knee_settings(skeleton, bone_chains_dict, fbik_settings)

        return {
            "template_name": template_used,
            "match_score": round(best_score, 4),
            "root_bone_dag": root_dag or "",
            "pelvis_bone_dag": pelvis_dag,
            "root_bone": skeleton.get_clean_name(root_dag) if root_dag else "",
            "pelvis_bone": skeleton.get_clean_name(pelvis_dag),
            "root_bone_clean": skeleton.get_clean_name(root_dag) if root_dag else "",
            "pelvis_bone_clean": skeleton.get_clean_name(pelvis_dag),
            "bones_to_exclude_from_auto_pose": excluded,
            "bone_chains": bone_chains_dict,
            "chains": chain_descriptors,
            "ik_goals": goals,
            "goals": goals,
            "fbik_bone_settings": fbik_settings,
            "bones_to_pin": bones_to_pin,
        }

