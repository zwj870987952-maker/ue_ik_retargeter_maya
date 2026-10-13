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
    """Strips both Maya DAG path ('|') and namespace (':') prefixes."""
    return str(node_name).split("|")[-1].split(":")[-1]


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
    """

    @staticmethod
    def save_to_dict(processor):
        """Serializes processor's source/target rest poses and aligned pose offsets to a dict."""
        src_skel = processor.source_skeleton
        tgt_skel = processor.target_skeleton
        pose_mgr = processor.target_pose_mgr

        src_data = {}
        for j in src_skel.joint_names:
            clean_name = src_skel.get_clean_name(j)
            src_data[clean_name] = {
                "dag_path": j,
                "rest_world_pos": _vec_to_list(src_skel.rest_world_pos[j]),
                "rest_world_rot": _quat_to_list(src_skel.rest_world_rot[j]),
                "rest_local_rot": _quat_to_list(src_skel.rest_local_rot[j]),
            }

        tgt_data = {}
        for j in tgt_skel.joint_names:
            clean_name = tgt_skel.get_clean_name(j)
            tgt_data[clean_name] = {
                "dag_path": j,
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
                "src_start": src_skel.get_clean_name(mapping.source_chain.start_bone),
                "src_end": src_skel.get_clean_name(mapping.source_chain.end_bone),
                "tgt_start": tgt_skel.get_clean_name(mapping.target_chain.start_bone),
                "tgt_end": tgt_skel.get_clean_name(mapping.target_chain.end_bone),
                "enable_ik": mapping.enable_ik,
                "ik_goal_name": mapping.ik_goal_name,
            })

        return {
            "version": "2.0",
            "source_root_clean": src_skel.get_clean_name(processor.source_root),
            "target_root_clean": tgt_skel.get_clean_name(processor.target_root),
            "source_pelvis_clean": src_skel.get_clean_name(processor.source_pelvis),
            "target_pelvis_clean": tgt_skel.get_clean_name(processor.target_pelvis),
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
            clean_name = src_skel.get_clean_name(j)
            short_name = _strip_ns(j)
            entry = src_cache.get(clean_name) or src_cache.get(short_name)
            if entry:
                src_skel.rest_world_pos[j] = _list_to_vec(entry["rest_world_pos"])
                src_skel.rest_world_rot[j] = _list_to_quat(entry["rest_world_rot"])
                src_skel.rest_local_rot[j] = _list_to_quat(entry["rest_local_rot"])

        tgt_cache = data.get("target_skeleton", {})
        pose_mgr.delta_rotations.clear()

        from .math_utils import get_world_transform
        curr_tgt_root_pos, _ = get_world_transform(processor.target_root)
        tgt_root_clean = tgt_skel.get_clean_name(processor.target_root)
        tgt_root_short = _strip_ns(processor.target_root)
        root_entry = tgt_cache.get(tgt_root_clean) or tgt_cache.get(tgt_root_short)
        scene_offset = om.MVector(0, 0, 0)
        if root_entry:
            cached_root_pos = _list_to_vec(root_entry["rest_world_pos"])
            scene_offset = om.MVector(
                curr_tgt_root_pos.x - cached_root_pos.x,
                0.0,
                curr_tgt_root_pos.z - cached_root_pos.z
            )

        for j in tgt_skel.joint_names:
            clean_name = tgt_skel.get_clean_name(j)
            short_name = _strip_ns(j)
            entry = tgt_cache.get(clean_name) or tgt_cache.get(short_name)
            if entry:
                tgt_skel.rest_world_pos[j] = _list_to_vec(entry["rest_world_pos"]) + scene_offset
                tgt_skel.rest_world_rot[j] = _list_to_quat(entry["rest_world_rot"])
                tgt_skel.rest_local_rot[j] = _list_to_quat(entry["rest_local_rot"])
                if "delta_rotation" in entry:
                    pose_mgr.delta_rotations[j] = _list_to_quat(entry["delta_rotation"])

        if "root_translation_offset" in data:
            pose_mgr.root_translation_offset = _list_to_vec(data["root_translation_offset"])

        for mapping in processor.chain_mappings:
            mapping.source_chain._build_chain()
            mapping.target_chain._build_chain()

        pose_mgr.recompute_aligned_transforms()

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
