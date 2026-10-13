# -*- coding: utf-8 -*-
"""
Retarget Pose & Alignment Cache Manager.
Serializes and deserializes pristine bind-pose (T-Pose / A-Pose) reference data
and auto-aligned retargeting offsets for Source (A) and Target (B) skeletons.
Eliminates any risk of animation keyframes polluting the reference base pose.
"""

import json
import os
import maya.api.OpenMaya as om


def _strip_ns(node_name):
    """Strips Maya namespace prefix so cached bone data works across any namespace."""
    return node_name.split(':')[-1]


def _vec_to_list(v):
    return [float(v.x), float(v.y), float(v.z)]


def _list_to_vec(lst):
    return om.MVector(float(lst[0]), float(lst[1]), float(lst[2]))


def _quat_to_list(q):
    return [float(q.x), float(q.y), float(q.z), float(q.w)]


def _list_to_quat(lst):
    return om.MQuaternion(float(lst[0]), float(lst[1]), float(lst[2]), float(lst[3]))


class RetargetCache(object):
    """
    Manages saving and loading of clean A -> B bind pose retargeting data (.retarget.json).
    Workflow:
      Step 1: Import clean A and B skeletons -> Generate & Save Retarget Cache.
      Step 2: Apply animation to A -> Load/Use Retarget Cache to bake animation onto B.
    """

    @staticmethod
    def save_to_dict(processor):
        """Serializes processor's source/target rest poses and aligned pose offsets to a dict."""
        src_skel = processor.source_skeleton
        tgt_skel = processor.target_skeleton
        pose_mgr = processor.target_pose_mgr

        src_data = {}
        for j in src_skel.joint_names:
            clean_name = _strip_ns(j)
            src_data[clean_name] = {
                "rest_world_pos": _vec_to_list(src_skel.rest_world_pos[j]),
                "rest_world_rot": _quat_to_list(src_skel.rest_world_rot[j]),
                "rest_local_rot": _quat_to_list(src_skel.rest_local_rot[j]),
            }

        tgt_data = {}
        for j in tgt_skel.joint_names:
            clean_name = _strip_ns(j)
            tgt_data[clean_name] = {
                "rest_world_pos": _vec_to_list(tgt_skel.rest_world_pos[j]),
                "rest_world_rot": _quat_to_list(tgt_skel.rest_world_rot[j]),
                "rest_local_rot": _quat_to_list(tgt_skel.rest_local_rot[j]),
                "aligned_world_pos": _vec_to_list(pose_mgr.get_aligned_rest_position(j)),
                "aligned_world_rot": _quat_to_list(pose_mgr.get_aligned_rest_rotation(j)),
            }
            if j in pose_mgr.delta_rotations:
                tgt_data[clean_name]["delta_rotation"] = _quat_to_list(pose_mgr.delta_rotations[j])

        chains_data = []
        for mapping in processor.chain_mappings:
            chains_data.append({
                "name": mapping.name,
                "src_start": _strip_ns(mapping.source_chain.start_bone),
                "src_end": _strip_ns(mapping.source_chain.end_bone),
                "tgt_start": _strip_ns(mapping.target_chain.start_bone),
                "tgt_end": _strip_ns(mapping.target_chain.end_bone),
                "enable_ik": mapping.enable_ik,
            })

        return {
            "version": "1.0",
            "source_root_clean": _strip_ns(processor.source_root),
            "target_root_clean": _strip_ns(processor.target_root),
            "source_pelvis_clean": _strip_ns(processor.source_pelvis),
            "target_pelvis_clean": _strip_ns(processor.target_pelvis),
            "root_translation_offset": _vec_to_list(pose_mgr.root_translation_offset),
            "source_skeleton": src_data,
            "target_skeleton": tgt_data,
            "chain_mappings": chains_data,
        }

    @staticmethod
    def save_to_file(processor, filepath):
        """Saves retarget bind-pose cache to a JSON file on disk."""
        data = RetargetCache.save_to_dict(processor)
        folder = os.path.dirname(os.path.abspath(filepath))
        if folder and not os.path.exists(folder):
            os.makedirs(folder, exist_ok=True)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return filepath

    @staticmethod
    def apply_dict_to_processor(processor, data):
        """
        Applies cached bind-pose transforms and alignment deltas onto an active processor,
        overriding any animation-polluted rest transforms.
        """
        src_skel = processor.source_skeleton
        tgt_skel = processor.target_skeleton
        pose_mgr = processor.target_pose_mgr

        src_cache = data.get("source_skeleton", {})
        for j in src_skel.joint_names:
            clean_name = _strip_ns(j)
            if clean_name in src_cache:
                entry = src_cache[clean_name]
                src_skel.rest_world_pos[j] = _list_to_vec(entry["rest_world_pos"])
                src_skel.rest_world_rot[j] = _list_to_quat(entry["rest_world_rot"])
                src_skel.rest_local_rot[j] = _list_to_quat(entry["rest_local_rot"])

        tgt_cache = data.get("target_skeleton", {})
        pose_mgr.delta_rotations.clear()

        # Check current target root horizontal offset in scene vs cached rest root position
        # If the user moved Target B sideways (e.g. translateX = 130) after caching, preserve that scene offset!
        from .math_utils import get_world_transform
        curr_tgt_root_pos, _ = get_world_transform(processor.target_root)
        tgt_root_clean = _strip_ns(processor.target_root)
        scene_offset = om.MVector(0, 0, 0)
        if tgt_root_clean in tgt_cache:
            cached_root_pos = _list_to_vec(tgt_cache[tgt_root_clean]["rest_world_pos"])
            # Preserve horizontal X/Z placement offset in the current scene
            scene_offset = om.MVector(
                curr_tgt_root_pos.x - cached_root_pos.x,
                0.0,
                curr_tgt_root_pos.z - cached_root_pos.z
            )

        for j in tgt_skel.joint_names:
            clean_name = _strip_ns(j)
            if clean_name in tgt_cache:
                entry = tgt_cache[clean_name]
                tgt_skel.rest_world_pos[j] = _list_to_vec(entry["rest_world_pos"]) + scene_offset
                tgt_skel.rest_world_rot[j] = _list_to_quat(entry["rest_world_rot"])
                tgt_skel.rest_local_rot[j] = _list_to_quat(entry["rest_local_rot"])
                if "delta_rotation" in entry:
                    pose_mgr.delta_rotations[j] = _list_to_quat(entry["delta_rotation"])

        if "root_translation_offset" in data:
            pose_mgr.root_translation_offset = _list_to_vec(data["root_translation_offset"])

        # Rebuild chain lengths and parameters from the restored clean rest positions
        for mapping in processor.chain_mappings:
            mapping.source_chain._build_chain()
            mapping.target_chain._build_chain()

        # Recompute target aligned transforms and refresh all solvers
        pose_mgr.recompute_aligned_transforms()

        # Re-initialize PelvisMotionSolver with restored rest poses
        from .pelvis_solver import PelvisMotionSolver
        processor.pelvis_solver = PelvisMotionSolver(
            processor.source_pelvis,
            processor.target_pelvis,
            processor.source_skeleton,
            processor.target_skeleton,
            processor.target_pose_mgr
        )
        processor.update_retarget_poses()

    @staticmethod
    def load_from_file(processor, filepath):
        """Loads a .retarget.json file from disk and applies it to the processor."""
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        RetargetCache.apply_dict_to_processor(processor, data)
        return data
