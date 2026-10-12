# -*- coding: utf-8 -*-
"""
Automatic pose alignment and ground snapping.
Directly implements UE5's FRetargetAutoPoseGenerator (IKRetargeterPoseGenerator.cpp).
Automatically aligns A-Pose to T-Pose (and vice versa) using chain tangent matching.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import quat_between_vectors, get_world_transform, set_world_transform


class AutoPoseAligner(object):
    """
    Analyzes anatomical chain directions on source and target skeletons,
    and automatically computes alignment rotation offsets so that
    different base poses (e.g. A-Pose vs T-Pose) match seamlessly without manual tuning.
    """

    @staticmethod
    def align_chains(processor):
        """
        Automatically aligns all mapped target chains to match source rest pose directions.
        Parallels UE5's FRetargetAutoPoseGenerator::AlignBones(ChainToChain).
        """
        pose_mgr = processor.target_pose_mgr
        pose_mgr.delta_rotations.clear()

        src_skel = processor.source_skeleton
        tgt_skel = processor.target_skeleton

        # Align each chain (Arms, Legs, Spine, Head)
        for mapping in processor.chain_mappings:
            src_chain = mapping.source_chain
            tgt_chain = mapping.target_chain

            if len(src_chain.bones) < 2 or len(tgt_chain.bones) < 2:
                continue

            # Calculate overall rest chain direction vector for source and target
            # Matches UE5: GetChainDirection(ChainName) = (ChainEnd - ChainStart).GetSafeNormal()
            src_start_pos = src_skel.rest_world_pos[src_chain.bones[0]]
            src_end_pos = src_skel.rest_world_pos[src_chain.bones[-1]]
            src_delta = src_end_pos - src_start_pos
            if src_delta.length() < 1e-4:
                continue
            src_dir = src_delta.normal()

            tgt_start_pos = tgt_skel.rest_world_pos[tgt_chain.bones[0]]
            tgt_end_pos = tgt_skel.rest_world_pos[tgt_chain.bones[-1]]
            tgt_delta = tgt_end_pos - tgt_start_pos
            if tgt_delta.length() < 1e-4:
                continue
            tgt_dir = tgt_delta.normal()

            # Find world rotation offset that rotates target chain direction to source chain direction
            # Matches UE5: FQuat WorldRotationOffset = FQuat::FindBetweenNormals(DirectionToAlign, DirectionToMatch);
            world_align_rot = quat_between_vectors(tgt_dir, src_dir)

            # Apply alignment rotation to target root bone of this chain (e.g. UpperArm / Thigh)
            root_bone = tgt_chain.bones[0]
            pose_mgr.delta_rotations[root_bone] = world_align_rot

        # Recompute full hierarchy transforms with the new alignment deltas
        pose_mgr.recompute_aligned_transforms()

        # Automatic Ground Snapping (Matches UE5 FRetargetAutoPoseGenerator::SnapToGround)
        AutoPoseAligner.snap_to_ground(processor)

        # Notify processor to update all solvers with the newly aligned pose
        if hasattr(processor, "update_retarget_poses"):
            processor.update_retarget_poses()

    @staticmethod
    def snap_to_ground(processor):
        """
        Ensures the target character's lowest foot joint touches the ground (Y=0),
        matching the source character's grounding height.
        """
        tgt_skel = processor.target_skeleton
        pose_mgr = processor.target_pose_mgr
        lowest_y = 999999.0
        lowest_bone = None

        for b in tgt_skel.joint_names:
            pos = pose_mgr.get_aligned_rest_position(b)
            if pos.y < lowest_y:
                lowest_y = pos.y
                lowest_bone = b

        if lowest_bone:
            # If lowest foot is offset from ground level (Y=0), compensate pelvis
            ground_offset = 0.0 - lowest_y
            pose_mgr.root_translation_offset = om.MVector(0, ground_offset, 0)
            pose_mgr.recompute_aligned_transforms()
