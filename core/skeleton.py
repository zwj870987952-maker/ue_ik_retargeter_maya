# -*- coding: utf-8 -*-
"""
Skeleton representation and transform caching for Maya animation retargeting.
Parallels UE 5.8's FRetargetSkeleton and FAbstractHierarchy.
Uses full Maya DAG paths internally so multiple skeletons with identical
joint names in the same Maya scene never collide.
"""

import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import get_world_transform, set_world_transform, euler_deg_to_quat


def resolve_skeleton_root_joint(node):
    """
    Given any selected node (joint, transform group, or mesh) in Maya,
    returns the full DAG path of the skeleton's root joint.
    """
    if not node or not cmds.objExists(node):
        return None

    long_nodes = cmds.ls(node, l=True)
    if not long_nodes:
        return None
    full_node = long_nodes[0]

    # If the selected node is already a joint, walk up to the top-most joint in its chain
    if cmds.nodeType(full_node) == "joint":
        curr = full_node
        while True:
            parent_joints = cmds.listRelatives(curr, p=True, type="joint", f=True) or []
            if parent_joints:
                curr = parent_joints[0]
            else:
                break
        return curr

    # Otherwise, search descendants for joints and pick the shallowest (top-most) joint
    desc_joints = cmds.listRelatives(full_node, ad=True, type="joint", f=True) or []
    if desc_joints:
        desc_joints.sort(key=lambda p: p.count("|"))
        return desc_joints[0]

    # If a mesh was selected, check if it has a skinCluster connected to joints
    shapes = cmds.listRelatives(full_node, s=True, f=True) or [full_node]
    for shp in shapes:
        history = cmds.listHistory(shp) or []
        skin_clusters = cmds.ls(history, type="skinCluster") or []
        for sc in skin_clusters:
            inf_joints = cmds.skinCluster(sc, q=True, inf=True) or []
            if inf_joints:
                return resolve_skeleton_root_joint(inf_joints[0])

    return None


def find_largest_common_prefix(names):
    """
    Parallels UE 5.8 FAbstractHierarchy::FindLargestCommonPrefix.
    Strips common prefixes only when shared across the entire skeleton (e.g. 'mixamorig_').
    """
    if not names or len(names) < 2:
        return ""
    sorted_names = sorted(names)
    first_s = sorted_names[0]
    last_s = sorted_names[-1]
    prefix_chars = []
    for i in range(min(len(first_s), len(last_s))):
        if first_s[i] == last_s[i]:
            prefix_chars.append(first_s[i])
        else:
            break
    prefix = "".join(prefix_chars)
    # Only strip if prefix ends with a delimiter like '_', ':', or '-'
    for delim in [":", "_", "-"]:
        if delim in prefix:
            idx = prefix.rfind(delim)
            return prefix[: idx + 1]
    return ""


class RetargetSkeleton(object):
    """
    Encapsulates a skeletal hierarchy in Maya using full DAG paths for 100%
    duplicate-name safety when two skeletons share identical joint names.
    """

    def __init__(self, root_node):
        resolved_root = resolve_skeleton_root_joint(root_node)
        if not resolved_root:
            raise RuntimeError("Could not resolve a valid root joint from: {}".format(root_node))

        self.root_joint = resolved_root  # Full DAG path, e.g. '|CharA|Hips'
        self.character_tag = self._derive_character_tag(resolved_root)

        self.joint_names = []       # Ordered list of full DAG paths (root to leaf)
        self.joint_indices = {}     # full_dag_path -> index
        self.parent_indices = []    # index -> parent_index (-1 for root)

        self.short_names = {}       # full_dag_path -> short Maya name (with namespace if any)
        self.clean_names = {}       # full_dag_path -> clean bone name (no namespace/prefix)
        self.clean_to_dag = {}      # clean_name -> full_dag_path
        self.lower_clean_to_dag = {}  # clean_name.lower() -> full_dag_path

        self.rest_world_pos = {}    # full_dag_path -> MVector
        self.rest_world_rot = {}    # full_dag_path -> MQuaternion
        self.rest_local_pos = {}    # full_dag_path -> MVector (in parent frame)
        self.rest_local_rot = {}    # full_dag_path -> MQuaternion (in parent frame)

        self._build_hierarchy()
        self.cache_reference_tpose()

    @staticmethod
    def _derive_character_tag(root_dag_path):
        """Derives a clean, unique identifier tag for this skeleton from its DAG path or namespace."""
        parts = [p for p in root_dag_path.split("|") if p]
        short_root = parts[-1]
        if ":" in short_root:
            ns = short_root.split(":")[0]
            if ns and ns.lower() != "mixamorig":
                return ns
        if len(parts) > 1:
            return parts[0].split(":")[-1]
        return short_root.replace(":", "_")

    def _build_hierarchy(self):
        """Discovers all joints under root_joint using full DAG paths and builds clean name maps."""
        all_joints = cmds.listRelatives(self.root_joint, ad=True, type="joint", f=True) or []
        all_joints.reverse()
        self.joint_names = [self.root_joint] + all_joints

        for idx, dag_path in enumerate(self.joint_names):
            self.joint_indices[dag_path] = idx

        self.parent_indices = [-1] * len(self.joint_names)
        for idx, dag_path in enumerate(self.joint_names):
            parent = cmds.listRelatives(dag_path, p=True, type="joint", f=True)
            if parent and parent[0] in self.joint_indices:
                self.parent_indices[idx] = self.joint_indices[parent[0]]
            else:
                self.parent_indices[idx] = -1

        # Build short names and strip namespaces
        no_ns_names = []
        for dag_path in self.joint_names:
            short = dag_path.split("|")[-1]
            self.short_names[dag_path] = short
            no_ns = short.split(":")[-1]
            no_ns_names.append(no_ns)

        # Strip common prefix if present across all bones (UE 5.8 GenerateCleanBoneNames)
        common_prefix = find_largest_common_prefix(no_ns_names)
        self.common_prefix = common_prefix
        for dag_path, no_ns in zip(self.joint_names, no_ns_names):
            clean = no_ns[len(common_prefix):] if (common_prefix and no_ns.startswith(common_prefix)) else no_ns
            if not clean:
                clean = no_ns
            self.clean_names[dag_path] = clean
            if clean not in self.clean_to_dag:
                self.clean_to_dag[clean] = dag_path
            if clean.lower() not in self.lower_clean_to_dag:
                self.lower_clean_to_dag[clean.lower()] = dag_path

    def resolve_bone_dag(self, bone_name_or_dag):
        """
        Resolves either a full DAG path, a short name, or a clean bone name
        to this skeleton's unique full DAG path.
        """
        if not bone_name_or_dag:
            return None
        if bone_name_or_dag in self.joint_indices:
            return bone_name_or_dag
        if bone_name_or_dag in self.clean_to_dag:
            return self.clean_to_dag[bone_name_or_dag]
        clean_key = bone_name_or_dag.split("|")[-1].split(":")[-1]
        if clean_key in self.clean_to_dag:
            return self.clean_to_dag[clean_key]
        if clean_key.lower() in self.lower_clean_to_dag:
            return self.lower_clean_to_dag[clean_key.lower()]
        return None

    def get_clean_name(self, dag_path):
        """Returns the stripped clean bone name for a DAG path."""
        return self.clean_names.get(dag_path, str(dag_path).split("|")[-1].split(":")[-1])

    def get_short_name(self, dag_path):
        """Returns the short Maya node name (without '|' DAG prefix) for a DAG path."""
        return self.short_names.get(dag_path, str(dag_path).split("|")[-1])

    def cache_reference_tpose(self, force_recapture=True):
        """
        Directly captures the skeleton's pose at the CURRENT frame as the reference/bind pose
        for building retarget assets (does NOT query Maya dagPose/bindPose or disconnect curves).
        """
        self.cache_rest_pose()

    def capture_current_frame_as_bind_pose(self):
        """
        Explicitly captures the current frame's world and local joint transforms
        as the reference bind pose.
        """
        self.cache_rest_pose()

    def cache_rest_pose(self):
        """
        Records the current frame's pose as the base reference/bind pose.
        """
        for idx, dag_path in enumerate(self.joint_names):
            pos, rot = get_world_transform(dag_path)
            self.rest_world_pos[dag_path] = pos
            self.rest_world_rot[dag_path] = rot.normal()
        self._compute_local_rest_transforms()

    def _compute_local_rest_transforms(self):
        """
        Computes parent-relative local rest transforms from rest_world_pos and rest_world_rot
        using Maya OpenMaya row-vector quaternion conventions:
            R_world = R_local * R_parent_world  =>  R_local = R_world * R_parent_world.inverse()
            P_local = (P_world - P_parent_world).rotateBy(R_parent_world.inverse())
        """

        for idx, dag_path in enumerate(self.joint_names):
            p_idx = self.parent_indices[idx]
            pos = self.rest_world_pos[dag_path]
            rot = self.rest_world_rot[dag_path]
            if p_idx == -1:
                self.rest_local_pos[dag_path] = om.MVector(pos)
                self.rest_local_rot[dag_path] = om.MQuaternion(rot)
            else:
                p_dag = self.joint_names[p_idx]
                p_pos = self.rest_world_pos[p_dag]
                p_rot = self.rest_world_rot[p_dag]
                # Local vector in parent's frame
                self.rest_local_pos[dag_path] = om.MVector(pos - p_pos).rotateBy(p_rot.inverse())
                # Local rotation in parent's frame (Maya row-vector order: R_local * R_parent = R_child)
                self.rest_local_rot[dag_path] = (rot * p_rot.inverse()).normal()

    def get_bone_index(self, joint_name_or_dag):
        dag = self.resolve_bone_dag(joint_name_or_dag)
        return self.joint_indices.get(dag, -1)

    def get_parent_joint(self, joint_name_or_dag):
        idx = self.get_bone_index(joint_name_or_dag)
        if idx >= 0:
            p_idx = self.parent_indices[idx]
            if p_idx >= 0:
                return self.joint_names[p_idx]
        return None

    def get_children_indices_recursive(self, start_bone_index):
        """Parallels UE 5.8 FRetargetSkeleton::GetChildrenIndicesRecursive."""
        children = []
        visited = {start_bone_index}
        for idx in range(start_bone_index + 1, len(self.joint_names)):
            p_idx = self.parent_indices[idx]
            if p_idx in visited:
                children.append(idx)
                visited.add(idx)
        return children

    def get_current_pose(self):
        """Returns dict of current world positions and rotations keyed by full DAG path."""
        pose = {}
        for dag_path in self.joint_names:
            pos, rot = get_world_transform(dag_path)
            pose[dag_path] = (pos, rot.normal())
        return pose

    def apply_pose(self, pose_dict):
        """Applies a dict of dag_path: (pos, rot) to Maya joints in topological order."""
        for dag_path in self.joint_names:
            if dag_path in pose_dict:
                pos, rot = pose_dict[dag_path]
                set_world_transform(dag_path, pos, rot)
