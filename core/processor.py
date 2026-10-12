# -*- coding: utf-8 -*-
"""
Main retarget processor coordinating pelvis, FK chains, and IK goals.
Directly implements UE5's FIKRetargetProcessor (IKRetargetProcessor.cpp).
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .skeleton import RetargetSkeleton
from .chains import BoneChain, ChainMapping, CHAIN_PATTERNS
from .retarget_pose import RetargetPoseManager
from .pelvis_solver import PelvisMotionSolver
from .fk_solver import FKChainSolver
from .ik_solver import IKChainSolver
from .math_utils import set_world_transform, get_world_transform, quat_between_vectors


class IKRetargetProcessor(object):
    """
    Master processor that converts an animated source skeleton pose
    into an anatomically accurate pose on a target skeleton.
    """

    def __init__(self, source_root, target_root, source_pelvis=None, target_pelvis=None):
        self.source_root = source_root
        self.target_root = target_root
        self.source_pelvis = source_pelvis or source_root
        self.target_pelvis = target_pelvis or target_root

        # Skeletons
        self.source_skeleton = RetargetSkeleton(self.source_root)
        self.target_skeleton = RetargetSkeleton(self.target_root)

        # Pose manager for A-Pose to T-Pose alignments
        self.target_pose_mgr = RetargetPoseManager(self.target_skeleton)

        # Solvers
        self.pelvis_solver = None
        self.chain_mappings = []
        self.fk_solvers = []
        self.ik_solvers = []

        self.setup_default_chains()

    def setup_default_chains(self):
        """Discovers standard chains and initializes default solvers."""
        self.pelvis_solver = PelvisMotionSolver(
            self.source_pelvis,
            self.target_pelvis,
            self.source_skeleton,
            self.target_skeleton,
            self.target_pose_mgr
        )
        self.chain_mappings.clear()
        self.fk_solvers.clear()
        self.ik_solvers.clear()

    def add_chain_mapping(self, name, src_start, src_end, tgt_start, tgt_end, enable_ik=False):
        """Adds a retarget chain mapping between source and target joints."""
        if not (cmds.objExists(src_start) and cmds.objExists(src_end) and
                cmds.objExists(tgt_start) and cmds.objExists(tgt_end)):
            return False

        src_chain = BoneChain(name + "_Src", src_start, src_end, self.source_skeleton)
        tgt_chain = BoneChain(name + "_Tgt", tgt_start, tgt_end, self.target_skeleton)

        mapping = ChainMapping(name, src_chain, tgt_chain, enable_ik=enable_ik)
        self.chain_mappings.append(mapping)

        # Create FK solver
        fk_solv = FKChainSolver(mapping, self.target_pose_mgr)
        self.fk_solvers.append(fk_solv)

        # Create IK solver if requested
        if enable_ik and len(tgt_chain.bones) >= 3:
            ik_solv = IKChainSolver(mapping, self.target_pose_mgr)
            self.ik_solvers.append(ik_solv)

        return True

    def auto_map_chains(self):
        """
        Fuzzy matches bones on source and target based on standard naming conventions.
        Supports Mixamo, UE Mannequin, AdvancedSkeleton, Biped.
        """
        self.chain_mappings.clear()
        self.fk_solvers.clear()
        self.ik_solvers.clear()

        src_joints = self.source_skeleton.joint_names
        tgt_joints = self.target_skeleton.joint_names

        def find_match(joint_list, pattern_list):
            for pat in pattern_list:
                clean_pat = pat.replace('*', '').lower()
                for j in joint_list:
                    j_clean = j.split(':')[-1].lower()  # Remove namespace
                    if clean_pat in j_clean:
                        return j
            return None

        # Auto-match pelvis
        pelvis_pats = CHAIN_PATTERNS['Pelvis']['patterns']
        src_pelvis_found = find_match(src_joints, pelvis_pats)
        tgt_pelvis_found = find_match(tgt_joints, pelvis_pats)
        if src_pelvis_found and tgt_pelvis_found:
            self.source_pelvis = src_pelvis_found
            self.target_pelvis = tgt_pelvis_found
            self.pelvis_solver = PelvisMotionSolver(
                self.source_pelvis, self.target_pelvis,
                self.source_skeleton, self.target_skeleton,
                self.target_pose_mgr
            )

        # Auto-match chains
        for chain_name, defs in CHAIN_PATTERNS.items():
            if chain_name == 'Pelvis':
                continue
            src_start = find_match(src_joints, defs['start'])
            src_end = find_match(src_joints, defs['end'])
            tgt_start = find_match(tgt_joints, defs['start'])
            tgt_end = find_match(tgt_joints, defs['end'])

            if src_start and src_end and tgt_start and tgt_end:
                enable_ik = defs.get('is_limb', False)
                self.add_chain_mapping(chain_name, src_start, src_end, tgt_start, tgt_end, enable_ik=enable_ik)

    def retarget_frame(self):
        """
        Processes a single frame:
        1. Samples animated source global pose.
        2. Solves pelvis motion (scaled translation + delta rotation).
        3. Solves FK chains rotation transfer.
        4. Solves IK goals and applies analytical Two-Bone IK on limbs.
        """
        # Step 1: Capture current source pose
        src_pose = self.source_skeleton.get_current_pose()
        src_pelvis_pos, src_pelvis_rot = src_pose[self.source_pelvis]

        # Step 2: Solve Pelvis
        tgt_pelvis_pos, tgt_pelvis_rot = self.pelvis_solver.solve(src_pelvis_pos, src_pelvis_rot)
        set_world_transform(self.target_pelvis, tgt_pelvis_pos, tgt_pelvis_rot)

        # Step 3: Solve FK Chains
        for fk_solv in self.fk_solvers:
            chain_rotations = fk_solv.solve(src_pose)
            for tgt_joint, world_rot in chain_rotations.items():
                set_world_transform(tgt_joint, quat=world_rot)

        # Step 4: Solve IK Chains (for limbs: LeftLeg, RightLeg, LeftArm, RightArm)
        for ik_solv in self.ik_solvers:
            tgt_chain = ik_solv.tgt_chain
            if len(tgt_chain.bones) < 3:
                continue

            # Start joint (e.g. Thigh) current world position
            root_bone = tgt_chain.bones[0]
            mid_bone = tgt_chain.bones[1]
            end_bone = tgt_chain.bones[2]

            tgt_start_pos, _ = get_world_transform(root_bone)
            goal_pos, goal_rot = ik_solv.calculate_goal_transform(src_pose, tgt_start_pos)

            # Get bone lengths
            p0 = self.target_skeleton.rest_world_pos[root_bone]
            p1 = self.target_skeleton.rest_world_pos[mid_bone]
            p2 = self.target_skeleton.rest_world_pos[end_bone]
            upper_len = (p1 - p0).length()
            lower_len = (p2 - p1).length()

            # Source pole vector hint
            src_root_pos, _ = src_pose[ik_solv.src_chain.bones[0]]
            src_mid_pos, _ = src_pose[ik_solv.src_chain.bones[1]]
            src_end_pos, _ = src_pose[ik_solv.src_chain.bones[2]]
            src_pv = (src_mid_pos - (src_root_pos + src_end_pos) * 0.5).normal()

            # Solve analytical 2-bone IK
            mid_pos = ik_solv.solve_two_bone_ik(tgt_start_pos, goal_pos, src_pv, upper_len, lower_len)

            # Orient root bone towards mid_pos
            curr_root_pos, curr_root_rot = get_world_transform(root_bone)
            aim_dir = (mid_pos - curr_root_pos).normal()
            rest_dir = (p1 - p0).normal()
            root_rot = quat_between_vectors(rest_dir, aim_dir) * curr_root_rot
            set_world_transform(root_bone, quat=root_rot)

            # Orient mid bone towards goal_pos
            curr_mid_pos, curr_mid_rot = get_world_transform(mid_bone)
            aim_dir2 = (goal_pos - curr_mid_pos).normal()
            rest_dir2 = (p2 - p1).normal()
            mid_rot = quat_between_vectors(rest_dir2, aim_dir2) * curr_mid_rot
            set_world_transform(mid_bone, quat=mid_rot)

            # Orient end bone to goal rotation
            set_world_transform(end_bone, quat=goal_rot)

    def bake_animation(self, start_frame, end_frame, step=1, target_controls=None):
        """
        Bakes retargeted animation across the specified frame range.
        If target_controls is provided, keys are set on controllers; otherwise on target skeleton.
        """
        keys_target = target_controls if target_controls else self.target_skeleton.joint_names

        orig_time = cmds.currentTime(q=True)
        try:
            for frame in range(int(start_frame), int(end_frame) + 1, int(step)):
                cmds.currentTime(frame, e=True)
                self.retarget_frame()

                # Set keyframes on all target bones
                for node in keys_target:
                    cmds.setKeyframe(node, time=frame, attribute=['translate', 'rotate'])
        finally:
            cmds.currentTime(orig_time, e=True)
