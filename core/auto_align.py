# -*- coding: utf-8 -*-
"""
Automatic pose alignment and ground snapping.
Directly implements UE 5.8's FRetargetAutoPoseGenerator (IKRetargeterPoseGenerator.cpp).
Automatically aligns A-Pose to T-Pose (and vice versa) using per-bone chain tangent matching
in topological root-to-leaf order, enforces BonesToExcludeFromAutoPose (flat feet),
and snaps the aligned character back to its original bind-pose ground height.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import quat_between_vectors


class AutoPoseAligner(object):
    """
    Analyzes anatomical chain directions on source and target skeletons,
    and automatically computes per-bone alignment rotation offsets so that
    different base poses (e.g. A-Pose vs T-Pose, bent elbows/knees vs straight)
    match seamlessly without manual tuning.
    """

    @staticmethod
    def align_chains(processor):
        """
        Automatically aligns all mapped target chains to match source rest pose directions.
        Parallels UE 5.8's FRetargetAutoPoseGenerator::AlignAllBones / AlignBones(ChainToChain).
        """
        pose_mgr = processor.target_pose_mgr
        pose_mgr.delta_rotations.clear()
        pose_mgr.local_delta_rotations.clear()
        pose_mgr.root_translation_offset = om.MVector(0.0, 0.0, 0.0)
        pose_mgr.recompute_aligned_transforms()

        src_skel = processor.source_skeleton
        tgt_skel = processor.target_skeleton

        # Collect bones excluded from auto pose (e.g. LeftFoot, RightFoot / foot_l, foot_r)
        excluded_dags = set()
        raw_excluded = getattr(processor, "bones_to_exclude_from_auto_pose", None) or []
        for b_item in raw_excluded:
            dag = tgt_skel.resolve_bone_dag(b_item)
            if dag:
                excluded_dags.add(dag)

        # Also auto-detect foot/ankle end bones on leg chains if not explicitly provided
        for mapping in processor.chain_mappings:
            if "leg" in mapping.name.lower() and len(mapping.target_chain.bones) >= 3:
                excluded_dags.add(mapping.target_chain.bones[-1])

        # Sort chain mappings in topological order of target start bone index
        def _chain_topo_key(mapping):
            if not mapping.target_chain.bones:
                return 999999
            b0 = mapping.target_chain.bones[0]
            if b0 in tgt_skel.joint_names:
                return tgt_skel.joint_names.index(b0)
            return 999999

        sorted_mappings = sorted(processor.chain_mappings, key=_chain_topo_key)

        # Align each chain bone-by-bone in topological order (UE 5.8 AlignBone ChainToChain)
        for mapping in sorted_mappings:
            src_chain = mapping.source_chain
            tgt_chain = mapping.target_chain

            # Skip single-bone chains (UE 5.8 GetChainTangent skips chains with < 2 bones)
            if len(src_chain.bones) < 2 or len(tgt_chain.bones) < 2:
                continue

            num_tgt_bones = len(tgt_chain.bones)
            for i in range(num_tgt_bones - 1):
                tgt_bone = tgt_chain.bones[i]
                next_tgt_bone = tgt_chain.bones[i + 1]

                if tgt_bone in excluded_dags:
                    continue

                # Parameter interval [u0, u1] along target chain mapped onto source chain
                u0 = tgt_chain.params[i]
                u1 = tgt_chain.params[i + 1]
                if u1 - u0 < 1e-4:
                    continue

                # Source tangent direction in source retarget/rest pose
                src_p0, _ = src_chain.get_interpolated_transform_at_param(
                    u0, rest_pose=True, pose_mgr=getattr(processor, "source_pose_mgr", None)
                )
                src_p1, _ = src_chain.get_interpolated_transform_at_param(
                    u1, rest_pose=True, pose_mgr=getattr(processor, "source_pose_mgr", None)
                )
                src_delta = src_p1 - src_p0
                if src_delta.length() < 1e-4:
                    continue
                src_dir = src_delta.normal()

                # Target current aligned tangent direction (already reflects parent alignments!)
                tgt_p0 = pose_mgr.get_aligned_rest_position(tgt_bone)
                tgt_p1 = pose_mgr.get_aligned_rest_position(next_tgt_bone)
                tgt_delta = tgt_p1 - tgt_p0
                if tgt_delta.length() < 1e-4:
                    continue
                tgt_dir = tgt_delta.normal()

                # World rotation offset rotating target tangent to source tangent
                world_align_rot = quat_between_vectors(tgt_dir, src_dir)

                # Apply world delta to tgt_bone and immediately propagate down hierarchy
                existing_delta = pose_mgr.delta_rotations.get(tgt_bone, om.MQuaternion.kIdentity)
                pose_mgr.set_world_delta_for_bone(
                    tgt_bone, (existing_delta * world_align_rot).normal(), recompute=True
                )

        # Enforce BonesToExcludeFromAutoPose (e.g. LeftFoot, RightFoot):
        # Keep excluded foot/ankle bones at their original bind-pose world orientation
        # so the soles of the feet stay flat on the floor even if thighs/calves rotated.
        sorted_excluded = sorted(
            excluded_dags,
            key=lambda b: tgt_skel.joint_names.index(b) if b in tgt_skel.joint_names else 999999
        )
        for ex_bone in sorted_excluded:
            pose_mgr.delta_rotations.pop(ex_bone, None)
            pose_mgr.recompute_aligned_transforms()
            propagated_rot = pose_mgr.aligned_world_rot[ex_bone]
            rest_rot = tgt_skel.rest_world_rot[ex_bone]
            compensate_delta = (propagated_rot.inverse() * rest_rot).normal()
            pose_mgr.set_world_delta_for_bone(ex_bone, compensate_delta, recompute=True)

        # Automatic Ground Snapping (Matches UE 5.8 FRetargetAutoPoseGenerator::SnapToGround)
        AutoPoseAligner.snap_to_ground(processor)

        # Notify processor to update all solvers with the newly aligned pose
        if hasattr(processor, "update_retarget_poses"):
            processor.update_retarget_poses()

    @staticmethod
    def snap_to_ground(processor):
        """
        Matches UE 5.8 FRetargetAutoPoseGenerator::SnapToGround:
        Compares the lowest foot bone height in the original bind pose (ref_lowest_y)
        against its height in the newly aligned pose (aligned_lowest_y), and translates
        the target root vertically by (ref_lowest_y - aligned_lowest_y) so the character
        sits at its exact original ground level without sinking into or floating above the floor.
        """
        tgt_skel = processor.target_skeleton
        pose_mgr = processor.target_pose_mgr

        # First zero out vertical offset before measuring aligned foot height
        pose_mgr.root_translation_offset = om.MVector(0.0, 0.0, 0.0)
        pose_mgr.recompute_aligned_transforms()

        # Prefer foot/leg chain bones for ground measurement if available
        candidate_bones = []
        for mapping in processor.chain_mappings:
            cname = mapping.name.lower()
            if "leg" in cname or "foot" in cname:
                for b in mapping.target_chain.bones:
                    if b not in candidate_bones:
                        candidate_bones.append(b)
        if not candidate_bones:
            candidate_bones = list(tgt_skel.joint_names)

        ref_lowest_y = 999999.0
        aligned_lowest_y = 999999.0

        for b in candidate_bones:
            ref_pos = tgt_skel.rest_world_pos[b]
            aligned_pos = pose_mgr.get_aligned_rest_position(b)
            if ref_pos.y < ref_lowest_y:
                ref_lowest_y = ref_pos.y
            if aligned_pos.y < aligned_lowest_y:
                aligned_lowest_y = aligned_pos.y

        if ref_lowest_y < 999998.0 and aligned_lowest_y < 999998.0:
            ground_offset_y = ref_lowest_y - aligned_lowest_y
            pose_mgr.root_translation_offset = om.MVector(0.0, ground_offset_y, 0.0)
            pose_mgr.recompute_aligned_transforms()
