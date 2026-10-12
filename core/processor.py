# -*- coding: utf-8 -*-
"""
Main retarget processor coordinating pelvis, FK chains, and IK goals.
Directly implements UE5's FIKRetargetProcessor (IKRetargetProcessor.cpp).
Automates chain mapping, pose alignment, and baking with zero manual setup.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .skeleton import RetargetSkeleton
from .chains import BoneChain, ChainMapping, CHAIN_PATTERNS
from .retarget_pose import RetargetPoseManager
from .pelvis_solver import PelvisMotionSolver
from .fk_solver import FKChainSolver
from .ik_solver import IKChainSolver, TwoBoneIKSolver
from .auto_align import AutoPoseAligner
from .math_utils import set_world_transform, get_world_transform


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

        # Initialize Skeletons
        self.source_skeleton = RetargetSkeleton(self.source_root)
        self.target_skeleton = RetargetSkeleton(self.target_root)

        # Pose manager for A-Pose to T-Pose alignments
        self.target_pose_mgr = RetargetPoseManager(self.target_skeleton)

        # Solvers
        self.pelvis_solver = None
        self.chain_mappings = []
        self.fk_solvers = []
        self.ik_solvers = []

        # Auto-configure chains and auto-align poses out of the box
        self.setup_default_chains()

    def setup_default_chains(self):
        """Discovers standard chains, auto-aligns base pose, and initializes solvers."""
        self.pelvis_solver = PelvisMotionSolver(
            self.source_pelvis,
            self.target_pelvis,
            self.source_skeleton,
            self.target_skeleton,
            self.target_pose_mgr
        )
        self.auto_map_chains()
        self.auto_align_base_pose()
        self.update_retarget_poses()

    def update_retarget_poses(self):
        """Refreshes all solvers when the retarget base pose is aligned or modified."""
        if self.pelvis_solver:
            self.pelvis_solver.height_scale = self.target_skeleton.rest_world_pos[self.target_pelvis].y / max(
                1e-4, self.source_skeleton.rest_world_pos[self.source_pelvis].y
            )
        for fk in self.fk_solvers:
            fk.update_retarget_pose()
        for ik in self.ik_solvers:
            ik.update_retarget_pose()

    def auto_map_chains(self):
        """
        Fuzzy matches bones on source and target based on standard naming conventions.
        Supports Mixamo, UE Mannequin, AdvancedSkeleton, Biped, DAZ, HumanIK.
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
                    j_clean = j.split(':')[-1].lower()  # Strip namespace
                    if clean_pat in j_clean:
                        return j
            return None

        # Auto-match Pelvis
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

        # Auto-match Chains
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

    def auto_align_base_pose(self):
        """
        Automatically aligns A-Pose to T-Pose (and vice-versa) using UE5 ChainToChain tangents.
        Fully automated: no manual viewport joint rotation needed.
        """
        AutoPoseAligner.align_chains(self)

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

    def retarget_frame(self):
        """
        Executes complete retarget pipeline for current frame:
        1. Samples animated source global pose.
        2. Solves pelvis motion (scaled translation + delta rotation).
        3. Solves FK chains rotation transfer in topological parent-to-child order.
        4. Solves limb IK goals and applies analytical Two-Bone IK with zero twist stripping.
        """
        # Step 1: Capture current source pose
        src_pose = self.source_skeleton.get_current_pose()
        src_pelvis_pos, src_pelvis_rot = src_pose[self.source_pelvis]

        # Step 2: Solve Pelvis
        tgt_pelvis_pos, tgt_pelvis_rot = self.pelvis_solver.solve(src_pelvis_pos, src_pelvis_rot)
        set_world_transform(self.target_pelvis, tgt_pelvis_pos, tgt_pelvis_rot)

        # Step 3: Solve FK Chains (Spine, Neck, Head, Base limb rotation)
        for fk_solv in self.fk_solvers:
            chain_rotations = fk_solv.solve(src_pose)
            for tgt_joint in fk_solv.tgt_chain.bones:
                if tgt_joint in chain_rotations:
                    set_world_transform(tgt_joint, quat=chain_rotations[tgt_joint])

        # Step 4: Solve IK Chains (Limbs: LeftLeg, RightLeg, LeftArm, RightArm)
        for ik_solv in self.ik_solvers:
            tgt_chain = ik_solv.tgt_chain
            if len(tgt_chain.bones) < 3:
                continue

            root_bone = tgt_chain.bones[0]  # Thigh / UpperArm
            mid_bone = tgt_chain.bones[1]   # Calf / ForeArm
            end_bone = tgt_chain.bones[2]   # Foot / Hand

            # Target start joint current world position and FK rotations
            tgt_start_pos, current_root_rot = get_world_transform(root_bone)
            current_mid_pos, current_mid_rot = get_world_transform(mid_bone)
            current_end_pos, current_end_rot = get_world_transform(end_bone)

            goal_pos, goal_rot = ik_solv.calculate_goal_transform(src_pose, tgt_start_pos)

            # Target limb lengths from aligned rest pose
            p0 = self.target_pose_mgr.get_aligned_rest_position(root_bone)
            p1 = self.target_pose_mgr.get_aligned_rest_position(mid_bone)
            p2 = self.target_pose_mgr.get_aligned_rest_position(end_bone)
            len1 = (p1 - p0).length()
            len2 = (p2 - p1).length()

            # Source Pole Vector hint (knee/elbow bend plane)
            src_root_b = ik_solv.src_chain.bones[0]
            src_mid_b = ik_solv.src_chain.bones[1]
            src_end_b = ik_solv.src_chain.bones[2]
            src_p0, _ = src_pose[src_root_b]
            src_p1, _ = src_pose[src_mid_b]
            src_p2, _ = src_pose[src_end_b]
            src_pv_delta = (src_p1 - (src_p0 + src_p2) * 0.5)

            if src_pv_delta.length() > 1e-4:
                pv_target = tgt_start_pos + src_pv_delta.normal() * len1
            else:
                pv_target = current_mid_pos

            # End bone orientation blend:
            # For feet: blend=1.0 to match goal_rot (floor alignment).
            # For hands: blend=0.0 so wrist naturally follows forearm delta without popping.
            is_leg = ("leg" in ik_solv.mapping.name.lower()) or ("foot" in end_bone.lower())
            end_rot_blend = 1.0 if is_leg else 0.0

            # Solve Two-Bone IK with UE5 Delta Rotation preservation
            rot_upper, rot_lower, rot_end, _, _ = TwoBoneIKSolver.solve(
                tgt_start_pos, current_mid_pos, current_end_pos,
                current_root_rot, current_mid_rot, current_end_rot,
                goal_pos, goal_rot,
                pv_target,
                len1, len2,
                end_rot_blend=end_rot_blend
            )

            # Apply IK rotations
            set_world_transform(root_bone, quat=rot_upper)
            set_world_transform(mid_bone, quat=rot_lower)
            set_world_transform(end_bone, quat=rot_end)

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

                # Set clean keyframes
                for node in keys_target:
                    cmds.setKeyframe(node, time=frame, attribute=['translate', 'rotate'])
        finally:
            cmds.currentTime(orig_time, e=True)

    @classmethod
    def one_click_retarget(cls, source_root=None, target_root=None, start_frame=None, end_frame=None):
        """
        All-in-one entrypoint matching UE5's auto-retarget behavior.
        Automatically discovers characters, builds chains, aligns A/T-Pose, and bakes animation!
        """
        # If roots not passed, auto-detect from selection or scene
        if not source_root or not target_root:
            sel = cmds.ls(sl=True, type='joint')
            if len(sel) >= 2:
                source_root = sel[0]
                target_root = sel[1]
            else:
                # Find top-level root joints in scene
                all_joints = cmds.ls(type='joint')
                roots = [j for j in all_joints if not cmds.listRelatives(j, p=True, type='joint')]
                if len(roots) >= 2:
                    source_root = roots[0]
                    target_root = roots[1]
                else:
                    raise RuntimeError("Please select or specify Source Root and Target Root joints.")

        if start_frame is None:
            start_frame = int(cmds.playbackOptions(q=True, minTime=True))
        if end_frame is None:
            end_frame = int(cmds.playbackOptions(q=True, maxTime=True))

        print("[UE5 Retargeter] Initializing processor for: {} -> {}".format(source_root, target_root))
        processor = cls(source_root, target_root)

        print("[UE5 Retargeter] Auto-aligning base poses (A-to-T Pose match)...")
        processor.auto_align_base_pose()
        processor.update_retarget_poses()

        print("[UE5 Retargeter] Baking animation from frame {} to {}...".format(start_frame, end_frame))
        processor.bake_animation(start_frame, end_frame)

        print("[UE5 Retargeter] One-click retarget completed successfully!")
        return processor
