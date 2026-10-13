# -*- coding: utf-8 -*-
"""
Real-Time Full-Body IK (FBIK) Solver for Maya.
Mirrors Unreal Engine 5.8's FIKRigFullBodyIKSolver (PBIK) behavior:
- Coupled Pelvis weight shift, tilt, and leg-tethered squatting
- Multi-bone progressive Spine bending (pitch/roll) and axial twisting (yaw)
- Clavicle / Shoulder auto-reach with stiffness compliance
- 4-Limb analytical Two-Bone IK with Preferred Angles & Pole Vector stability
- Live viewport interaction via OpenMaya MNodeMessage callbacks + Maya DG Expression
"""

import math
import maya.cmds as cmds
import maya.api.OpenMaya as om

from .math_utils import (
    get_world_transform,
    set_world_transform,
    quat_between_vectors,
    slerp,
    rotate_vector_by_quat,
)
from .ik_solver import TwoBoneIKSolver

# Global registry of active scene FBIK solvers and their OpenMaya callback IDs
_ACTIVE_FBIK_SOLVERS = {}
_ACTIVE_CALLBACK_IDS = {}


def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


def _clamp_quat_angle(quat, max_angle_rad):
    """Clamps the rotation angle of an MQuaternion to max_angle_rad."""
    q = om.MQuaternion(quat).normal()
    if q.w < 0.0:
        q = om.MQuaternion(-q.x, -q.y, -q.z, -q.w)
    w_clamped = _clamp(q.w, -1.0, 1.0)
    angle = 2.0 * math.acos(w_clamped)
    if angle <= max_angle_rad or angle < 1e-6:
        return q
    ratio = max_angle_rad / angle
    return slerp(om.MQuaternion.kIdentity, q, ratio).normal()


class FullBodyIKSolver(object):
    """
    Solves Full-Body IK across Pelvis, Spine, Clavicles, Neck/Head, and all 4 Limbs
    driven by the IK Effector Goal locators in the Maya scene.
    """

    def __init__(self, solver_key, skeleton, char_profile, pose_mgr=None):
        self.solver_key = solver_key
        self.skeleton = skeleton
        self.char_profile = char_profile
        self.pose_mgr = pose_mgr
        self._is_solving = False

        # Goal locator mapping: goal_name -> {"locator_dag": str, "bone_dag": str, "init_pos": MVector, "init_rot": MQuaternion}
        self.goal_locators = {}
        self.pelvis_locator_dag = None
        self.pelvis_init_pos = None
        self.pelvis_init_rot = None

        # Cache base pose (either aligned RetargetPose on Target B or BindPose on Source A)
        self.base_world_pos = {}
        self.base_world_rot = {}
        self.base_local_pos = {}
        self.base_local_rot = {}
        self.refresh_base_pose()

        # Resolve anatomical chains & bones
        self.pelvis_dag = char_profile.get("pelvis_bone_dag") or skeleton.root_joint
        self.bone_chains = char_profile.get("bone_chains", {})
        self.fbik_settings = char_profile.get("fbik_bone_settings", {})

        self.spine_bones = self._get_chain_bones("Spine")
        self.neck_bones = self._get_chain_bones("Neck")
        self.head_bones = self._get_chain_bones("Head")
        self.left_clav_dag = self._get_first_bone("LeftClavicle")
        self.right_clav_dag = self._get_first_bone("RightClavicle")
        self.left_arm_bones = self._get_chain_bones("LeftArm")
        self.right_arm_bones = self._get_chain_bones("RightArm")
        self.left_leg_bones = self._get_chain_bones("LeftLeg")
        self.right_leg_bones = self._get_chain_bones("RightLeg")

        # Precompute limb segment lengths in base pose
        self.left_arm_lengths = self._calc_limb_lengths(self.left_arm_bones)
        self.right_arm_lengths = self._calc_limb_lengths(self.right_arm_bones)
        self.left_leg_lengths = self._calc_limb_lengths(self.left_leg_bones)
        self.right_leg_lengths = self._calc_limb_lengths(self.right_leg_bones)

        # Precompute preferred bend directions in base pose for knees and elbows
        self.limb_bend_hints = {}
        self._cache_default_bend_hints()

    def _get_chain_bones(self, chain_name):
        ch = self.bone_chains.get(chain_name)
        return list(ch.bones) if (ch and ch.bones) else []

    def _get_first_bone(self, chain_name):
        bones = self._get_chain_bones(chain_name)
        return bones[0] if bones else None

    def _calc_limb_lengths(self, bones):
        if len(bones) < 3:
            return (0.0, 0.0, 1.0)
        p0 = self.base_world_pos[bones[0]]
        p1 = self.base_world_pos[bones[1]]
        p2 = self.base_world_pos[bones[2]]
        l1 = (p1 - p0).length()
        l2 = (p2 - p1).length()
        return (l1, l2, max(1e-4, l1 + l2))

    def _cache_default_bend_hints(self):
        """
        Caches natural anatomical pole-vector bend directions (in parent/root space)
        for arms (elbows backward/outward) and legs (knees forward) so straight limbs
        never bend backward when IK compresses them.
        """
        # Determine character forward axis from foot toe vector (or default +Z)
        char_fwd = om.MVector(0.0, 0.0, 1.0)
        for foot_chain_name, leg_bones in [("LeftFoot", self.left_leg_bones), ("RightFoot", self.right_leg_bones)]:
            ft_bones = self._get_chain_bones(foot_chain_name)
            if ft_bones and leg_bones:
                ankle_pos = self.base_world_pos[leg_bones[-1]]
                toe_pos = self.base_world_pos[ft_bones[-1]]
                horiz = om.MVector(toe_pos.x - ankle_pos.x, 0.0, toe_pos.z - ankle_pos.z)
                if horiz.length() > 1e-3:
                    char_fwd = horiz.normal()
                    break

        for limb_name, bones, is_leg in [
            ("LeftArm", self.left_arm_bones, False),
            ("RightArm", self.right_arm_bones, False),
            ("LeftLeg", self.left_leg_bones, True),
            ("RightLeg", self.right_leg_bones, True),
        ]:
            if len(bones) < 3:
                continue
            p0 = self.base_world_pos[bones[0]]
            p1 = self.base_world_pos[bones[1]]
            p2 = self.base_world_pos[bones[2]]
            chord = p2 - p0
            chord_dir = chord.normal() if chord.length() > 1e-4 else om.MVector(0, -1, 0)
            mid_vec = p1 - p0
            perp = mid_vec - chord_dir * (mid_vec * chord_dir)
            if perp.length() > 0.5:
                bend_world = perp.normal()
            else:
                # Fallback anatomical bend direction: Knees bend forward (+char_fwd), Elbows bend backward (-char_fwd)
                bend_world = char_fwd if is_leg else (-char_fwd)
            self.limb_bend_hints[limb_name] = bend_world

    def refresh_base_pose(self):
        """Caches the base world and local transforms for all joints."""
        self.base_world_pos.clear()
        self.base_world_rot.clear()
        self.base_local_pos.clear()
        self.base_local_rot.clear()

        for idx, dag in enumerate(self.skeleton.joint_names):
            if self.pose_mgr is not None:
                pos = self.pose_mgr.get_aligned_rest_position(dag)
                rot = self.pose_mgr.get_aligned_rest_rotation(dag)
            else:
                pos = self.skeleton.rest_world_pos[dag]
                rot = self.skeleton.rest_world_rot[dag]
            self.base_world_pos[dag] = om.MVector(pos)
            self.base_world_rot[dag] = om.MQuaternion(rot).normal()

        for idx, dag in enumerate(self.skeleton.joint_names):
            p_idx = self.skeleton.parent_indices[idx]
            pos = self.base_world_pos[dag]
            rot = self.base_world_rot[dag]
            if p_idx == -1:
                self.base_local_pos[dag] = om.MVector(pos)
                self.base_local_rot[dag] = om.MQuaternion(rot)
            else:
                p_dag = self.skeleton.joint_names[p_idx]
                p_pos = self.base_world_pos[p_dag]
                p_rot = self.base_world_rot[p_dag]
                self.base_local_pos[dag] = rotate_vector_by_quat(pos - p_pos, p_rot.inverse())
                self.base_local_rot[dag] = (rot * p_rot.inverse()).normal()

    def register_goal_locator(self, goal_name, locator_dag, bone_dag):
        """Registers an in-scene IK Effector Goal locator with this FBIK solver."""
        pos, rot = get_world_transform(locator_dag)
        self.goal_locators[goal_name] = {
            "locator_dag": locator_dag,
            "bone_dag": bone_dag,
            "init_pos": om.MVector(pos),
            "init_rot": om.MQuaternion(rot).normal(),
        }

    def register_pelvis_locator(self, locator_dag):
        """Registers an in-scene Pelvis root goal locator with this FBIK solver."""
        pos, rot = get_world_transform(locator_dag)
        self.pelvis_locator_dag = locator_dag
        self.pelvis_init_pos = om.MVector(pos)
        self.pelvis_init_rot = om.MQuaternion(rot).normal()

    def _propagate_hierarchy(self, world_pos, world_rot, local_rot_deltas=None):
        """
        Recomputes forward kinematic world positions and rotations for all joints
        below the pelvis in topological order.
        """
        pelvis_idx = self.skeleton.joint_indices.get(self.pelvis_dag, 0)
        for idx in range(pelvis_idx + 1, len(self.skeleton.joint_names)):
            dag = self.skeleton.joint_names[idx]
            p_idx = self.skeleton.parent_indices[idx]
            if p_idx < 0:
                continue
            p_dag = self.skeleton.joint_names[p_idx]
            if p_dag not in world_pos:
                continue
            p_pos = world_pos[p_dag]
            p_rot = world_rot[p_dag]

            loc_p = self.base_local_pos[dag]
            loc_r = self.base_local_rot[dag]

            world_pos[dag] = p_pos + rotate_vector_by_quat(loc_p, p_rot)
            prop_rot = (loc_r * p_rot).normal()
            if local_rot_deltas and dag in local_rot_deltas:
                prop_rot = (prop_rot * local_rot_deltas[dag]).normal()
            world_rot[dag] = prop_rot

    def solve_from_scene_goals(self):
        """
        Reads current transforms of all registered Goal locators in the Maya scene
        and solves the Full-Body IK pose onto the skeleton joints in real time.
        """
        if self._is_solving:
            return
        self._is_solving = True
        try:
            self._solve_internal()
        except Exception as exc:
            print("[UE5.8 FBIK Warning] {}".format(exc))
        finally:
            self._is_solving = False

    def _solve_internal(self):
        # 1. Sample current Goal locator world transforms from Maya scene
        curr_goals = {}
        goal_deltas_pos = {}
        for g_name, g_info in self.goal_locators.items():
            loc_dag = g_info["locator_dag"]
            if not cmds.objExists(loc_dag):
                continue
            pos, rot = get_world_transform(loc_dag)
            pos_alpha = cmds.getAttr(loc_dag + ".positionAlpha") if cmds.attributeQuery("positionAlpha", node=loc_dag, exists=True) else 1.0
            rot_alpha = cmds.getAttr(loc_dag + ".rotationAlpha") if cmds.attributeQuery("rotationAlpha", node=loc_dag, exists=True) else 1.0
            blended_pos = g_info["init_pos"] + (pos - g_info["init_pos"]) * pos_alpha
            blended_rot = slerp(g_info["init_rot"], rot, rot_alpha).normal()
            curr_goals[g_name] = (blended_pos, blended_rot, pos_alpha, rot_alpha)
            goal_deltas_pos[g_name] = blended_pos - g_info["init_pos"]

        if not curr_goals:
            return

        # Working pose dictionaries initialized to base pose
        world_pos = {dag: om.MVector(p) for dag, p in self.base_world_pos.items()}
        world_rot = {dag: om.MQuaternion(r) for dag, r in self.base_world_rot.items()}
        world_deltas = {}

        d_lh = goal_deltas_pos.get("LeftHandIK", om.MVector(0, 0, 0))
        d_rh = goal_deltas_pos.get("RightHandIK", om.MVector(0, 0, 0))
        d_lf = goal_deltas_pos.get("LeftFootIK", om.MVector(0, 0, 0))
        d_rf = goal_deltas_pos.get("RightFootIK", om.MVector(0, 0, 0))

        # Compute over-extension pull vectors when a goal is actively moved beyond comfortable limb reach
        def _calc_over_pull(goal_name, limb_bones, limb_lengths):
            if goal_name not in curr_goals or len(limb_bones) < 3:
                return om.MVector(0, 0, 0)
            g_delta = goal_deltas_pos.get(goal_name, om.MVector(0, 0, 0))
            if g_delta.length() < 1e-4:
                return om.MVector(0, 0, 0)
            g_pos = curr_goals[goal_name][0]
            sock_pos = self.base_world_pos[limb_bones[0]]
            total_l = max(1e-4, limb_lengths[2])
            vec = g_pos - sock_pos
            dist = vec.length()
            comfort_len = total_l * 0.78
            if dist > comfort_len and dist > 1e-4:
                activity = _clamp(g_delta.length() / (total_l * 0.15), 0.0, 1.0)
                return vec.normal() * ((dist - comfort_len) * activity)
            return om.MVector(0, 0, 0)

        pull_lh = _calc_over_pull("LeftHandIK", self.left_arm_bones, self.left_arm_lengths)
        pull_rh = _calc_over_pull("RightHandIK", self.right_arm_bones, self.right_arm_lengths)
        pull_lf = _calc_over_pull("LeftFootIK", self.left_leg_bones, self.left_leg_lengths)
        pull_rf = _calc_over_pull("RightFootIK", self.right_leg_bones, self.right_leg_lengths)

        # Manual pelvis locator offset (if PelvisRoot locator exists and was moved)
        pelvis_manual_dp = om.MVector(0, 0, 0)
        pelvis_manual_dq = om.MQuaternion.kIdentity
        if self.pelvis_locator_dag and cmds.objExists(self.pelvis_locator_dag):
            p_pos, p_rot = get_world_transform(self.pelvis_locator_dag)
            pelvis_manual_dp = p_pos - self.pelvis_init_pos
            pelvis_manual_dq = (self.pelvis_init_rot.inverse() * p_rot).normal()

        # Check if all goals are at their exact initial transforms (zero-drift guarantee at rest)
        any_goal_moved = (pelvis_manual_dp.length() > 1e-4) or (abs(pelvis_manual_dq.w) < 0.999999)
        if not any_goal_moved:
            for g_name, (b_pos, b_rot, _, _) in curr_goals.items():
                g_info = self.goal_locators[g_name]
                if (b_pos - g_info["init_pos"]).length() > 1e-4:
                    any_goal_moved = True
                    break
                dq = (g_info["init_rot"].inverse() * b_rot).normal()
                if abs(dq.w) < 0.999999:
                    any_goal_moved = True
                    break

        if not any_goal_moved:
            pelvis_idx = self.skeleton.joint_indices.get(self.pelvis_dag, 0)
            for idx in range(pelvis_idx, len(self.skeleton.joint_names)):
                dag = self.skeleton.joint_names[idx]
                if dag == self.pelvis_dag:
                    set_world_transform(dag, self.base_world_pos[dag], self.base_world_rot[dag])
                else:
                    set_world_transform(dag, quat=self.base_world_rot[dag])
            return

        # ======================================================================
        # STEP 1: Solve Pelvis Position & Orientation (Full-Body Weight Shift & Squat)
        # ======================================================================
        pelvis_base_pos = self.base_world_pos[self.pelvis_dag]
        pelvis_base_rot = self.base_world_rot[self.pelvis_dag]

        # Coupled translation from hands (ambient + over-extension pull) and feet
        pelvis_shift = (
            pelvis_manual_dp
            + (d_lh + d_rh) * 0.12
            + (pull_lh + pull_rh) * 0.45
            + (d_lf + d_rf) * 0.22
            + (pull_lf + pull_rf) * 0.65
        )
        new_pelvis_pos = pelvis_base_pos + pelvis_shift

        # Pelvis tilt rotation from upper/lower body pull (respecting rotation_stiffness ~ 0.95)
        pelvis_rot_delta = pelvis_manual_dq
        if self.spine_bones:
            chest_dag = self.spine_bones[-1]
            up_vec0 = (self.base_world_pos[chest_dag] - pelvis_base_pos)
            if up_vec0.length() > 1e-4:
                up_dir0 = up_vec0.normal()
                tilt_pull = (d_lh + d_rh) * 0.15 + (pull_lh + pull_rh) * 0.35 - (d_lf + d_rf) * 0.15
                up_dir1 = (up_vec0 + tilt_pull).normal()
                raw_pelvis_tilt = quat_between_vectors(up_dir0, up_dir1)
                clamped_tilt = _clamp_quat_angle(raw_pelvis_tilt, math.radians(18.0))
                pelvis_tilt = slerp(om.MQuaternion.kIdentity, clamped_tilt, 0.35).normal()
                pelvis_rot_delta = (pelvis_rot_delta * pelvis_tilt).normal()

        new_pelvis_rot = (pelvis_base_rot * pelvis_rot_delta).normal()

        # Exact Leg Tether Constraint: keep hip sockets within max_leg_reach of Foot IK Goals
        # so planted feet NEVER lift off or drift from their IK Goal locators when hands pull the body!
        for _ in range(4):
            for f_goal, leg_bones, leg_lens in [
                ("LeftFootIK", self.left_leg_bones, self.left_leg_lengths),
                ("RightFootIK", self.right_leg_bones, self.right_leg_lengths),
            ]:
                if f_goal in curr_goals and len(leg_bones) >= 3:
                    foot_g_pos = curr_goals[f_goal][0]
                    hip_local = self.base_local_pos[leg_bones[0]]
                    hip_world_offset = rotate_vector_by_quat(hip_local, new_pelvis_rot)
                    est_hip_pos = new_pelvis_pos + hip_world_offset
                    leg_vec = est_hip_pos - foot_g_pos
                    init_chord = (self.base_world_pos[leg_bones[0]] - self.base_world_pos[leg_bones[2]]).length()
                    max_leg_reach = max(init_chord, leg_lens[2] * 0.9995)
                    if leg_vec.length() > max_leg_reach and leg_vec.length() > 1e-4:
                        clamped_hip = foot_g_pos + leg_vec.normal() * max_leg_reach
                        new_pelvis_pos = clamped_hip - hip_world_offset

        world_pos[self.pelvis_dag] = new_pelvis_pos
        world_rot[self.pelvis_dag] = new_pelvis_rot
        self._propagate_hierarchy(world_pos, world_rot, world_deltas)

        # ======================================================================
        # STEP 2: Solve Spine Chain (Progressive Bending + Axial Twisting)
        # ======================================================================
        if self.spine_bones:
            spine_start = self.spine_bones[0]
            spine_end = self.spine_bones[-1]
            p_sp_start = world_pos[spine_start]
            p_sp_end = world_pos[spine_end]
            spine_vec = p_sp_end - p_sp_start
            spine_len = spine_vec.length()

            if spine_len > 1e-4:
                v0 = spine_vec.normal()
                # Chest leans toward both ambient hand motion and over-reach pull
                chest_pull = (d_lh + d_rh) * 0.28 + (pull_lh + pull_rh) * 0.75
                v1_vec = spine_vec + chest_pull
                v1 = v1_vec.normal() if v1_vec.length() > 1e-4 else v0

                q_bend = _clamp_quat_angle(quat_between_vectors(v0, v1), math.radians(48.0))

                # Axial Twist (Yaw around spine axis v1 when left/right hands pull asymmetrically)
                q_twist = om.MQuaternion.kIdentity
                if self.left_arm_bones and self.right_arm_bones:
                    r_l = self.base_world_pos[self.left_arm_bones[0]] - self.base_world_pos[spine_end]
                    r_r = self.base_world_pos[self.right_arm_bones[0]] - self.base_world_pos[spine_end]
                    f_l = d_lh * 0.38 + pull_lh * 0.75
                    f_r = d_rh * 0.38 + pull_rh * 0.75
                    torque_vec = (r_l ^ f_l) + (r_r ^ f_r)
                    arm_ref_len = max(20.0, self.left_arm_lengths[2])
                    twist_rad = _clamp((torque_vec * v1) / (arm_ref_len * 35.0), math.radians(-42.0), math.radians(42.0))
                    q_twist = om.MQuaternion(twist_rad, v1)

                q_spine_total = (q_bend * q_twist).normal()
                num_sp = len(self.spine_bones)
                per_bone_q = slerp(om.MQuaternion.kIdentity, q_spine_total, 1.0 / float(num_sp)).normal()

                for sp_bone in self.spine_bones:
                    world_deltas[sp_bone] = per_bone_q
                self._propagate_hierarchy(world_pos, world_rot, world_deltas)

        # ======================================================================
        # STEP 3: Solve Left & Right Clavicles (Shoulder Auto-Reach)
        # ======================================================================
        for clav_dag, arm_bones, arm_lens, goal_name in [
            (self.left_clav_dag, self.left_arm_bones, self.left_arm_lengths, "LeftHandIK"),
            (self.right_clav_dag, self.right_arm_bones, self.right_arm_lengths, "RightHandIK"),
        ]:
            if clav_dag and len(arm_bones) >= 3 and goal_name in curr_goals:
                shoulder_dag = arm_bones[0]
                if clav_dag == shoulder_dag:
                    continue
                p_clav = world_pos[clav_dag]
                p_shld = world_pos[shoulder_dag]
                g_pos = curr_goals[goal_name][0]
                init_g_pos = self.goal_locators[goal_name]["init_pos"]

                c_vec0 = init_g_pos - self.base_world_pos[clav_dag]
                c_vec1 = g_pos - p_clav
                rel_delta = c_vec1 - c_vec0
                if rel_delta.length() > 1e-4 and c_vec0.length() > 1e-4 and c_vec1.length() > 1e-4:
                    q_aim = quat_between_vectors(c_vec0.normal(), c_vec1.normal())
                    dist_ratio = _clamp((g_pos - p_shld).length() / max(1e-4, arm_lens[2]), 0.0, 1.5)
                    activity = _clamp(rel_delta.length() / (max(1e-4, arm_lens[2]) * 0.15), 0.0, 1.0)
                    clav_weight = _clamp((0.25 + 0.30 * max(0.0, dist_ratio - 0.5)) * activity, 0.0, 0.55)
                    q_clav = slerp(om.MQuaternion.kIdentity, q_aim, clav_weight)
                    q_clav = _clamp_quat_angle(q_clav, math.radians(30.0))
                    world_deltas[clav_dag] = q_clav
                    self._propagate_hierarchy(world_pos, world_rot, world_deltas)

        # ======================================================================
        # STEP 4: Solve 4 Limbs (Analytical Two-Bone IK + Pole Vectors + End Rot)
        # ======================================================================
        for limb_name, limb_bones, limb_lens, goal_name in [
            ("LeftArm", self.left_arm_bones, self.left_arm_lengths, "LeftHandIK"),
            ("RightArm", self.right_arm_bones, self.right_arm_lengths, "RightHandIK"),
            ("LeftLeg", self.left_leg_bones, self.left_leg_lengths, "LeftFootIK"),
            ("RightLeg", self.right_leg_bones, self.right_leg_lengths, "RightFootIK"),
        ]:
            if len(limb_bones) < 3 or goal_name not in curr_goals:
                continue

            b_upper, b_mid, b_end = limb_bones[0], limb_bones[1], limb_bones[2]
            g_pos, g_rot, pos_alpha, rot_alpha = curr_goals[goal_name]
            if pos_alpha < 1e-4:
                continue

            p0 = world_pos[b_upper]
            p1 = world_pos[b_mid]
            p2 = world_pos[b_end]
            r0 = world_rot[b_upper]
            r1 = world_rot[b_mid]
            r2 = world_rot[b_end]
            len1, len2, _ = limb_lens

            # Rotate default anatomical bend hint by pelvis/chest rotation delta so pole vectors follow character turn
            parent_dag = self.skeleton.get_parent_joint(b_upper) or self.pelvis_dag
            parent_delta = (self.base_world_rot[parent_dag].inverse() * world_rot[parent_dag]).normal()
            base_hint_dir = self.limb_bend_hints.get(limb_name, om.MVector(0, 0, 1))
            rotated_hint_dir = rotate_vector_by_quat(base_hint_dir, parent_delta)
            pv_hint_pos = p0 + rotated_hint_dir * len1

            solved_r0, solved_r1, solved_r2, out_p1, out_p2 = TwoBoneIKSolver.solve(
                p0, p1, p2,
                r0, r1, r2,
                g_pos, g_rot,
                pv_hint_pos,
                len1, len2,
                end_rot_blend=rot_alpha,
            )

            # Update world transforms for Upper, Mid, End, and propagate to child fingers/toes
            world_rot[b_upper] = solved_r0
            world_pos[b_mid] = out_p1
            world_rot[b_mid] = solved_r1
            world_pos[b_end] = out_p2
            world_rot[b_end] = solved_r2

            # Propagate from b_end ONLY to its actual descendants (fingers / toes)
            end_prefix = b_end + "|"
            end_idx = self.skeleton.joint_indices[b_end]
            for idx in range(end_idx + 1, len(self.skeleton.joint_names)):
                dag = self.skeleton.joint_names[idx]
                if not dag.startswith(end_prefix):
                    continue
                p_idx = self.skeleton.parent_indices[idx]
                if p_idx < 0:
                    continue
                p_dag = self.skeleton.joint_names[p_idx]
                if p_dag not in world_pos:
                    continue
                world_pos[dag] = world_pos[p_dag] + rotate_vector_by_quat(self.base_local_pos[dag], world_rot[p_dag])
                world_rot[dag] = (self.base_local_rot[dag] * world_rot[p_dag]).normal()

        # ======================================================================
        # STEP 5: Apply Solved Full-Body Pose to Maya Scene Joints
        # ======================================================================
        pelvis_idx = self.skeleton.joint_indices.get(self.pelvis_dag, 0)
        for idx in range(pelvis_idx, len(self.skeleton.joint_names)):
            dag = self.skeleton.joint_names[idx]
            if dag == self.pelvis_dag:
                set_world_transform(dag, world_pos[dag], world_rot[dag])
            else:
                set_world_transform(dag, quat=world_rot[dag])


def clear_fbik_callbacks(solver_key=None):
    """Removes registered OpenMaya attribute-changed callbacks for solver_key (or all solvers)."""
    keys = [solver_key] if solver_key else list(_ACTIVE_CALLBACK_IDS.keys())
    for k in keys:
        cb_ids = _ACTIVE_CALLBACK_IDS.pop(k, [])
        for cid in cb_ids:
            try:
                om.MMessage.removeCallback(cid)
            except Exception:
                pass
        _ACTIVE_FBIK_SOLVERS.pop(k, None)


def register_live_fbik_solver(solver, fbik_group_dag):
    """
    Wires up real-time Full-Body IK evaluation in Maya:
    1. Registers OpenMaya MNodeMessage.addAttributeChangedCallback on all Goal locators
       so interactive viewport dragging (Move/Rotate tool) updates the skeleton at 60+ FPS.
    2. Creates a Maya DG Expression node connecting all Goal locators' translate/rotate
       attributes to `fbik_group_dag.fbikEvalOutput`, ensuring DG connections are visible
       in the Node Editor and evaluated during timeline scrubbing.
    """
    key = solver.solver_key
    clear_fbik_callbacks(key)
    _ACTIVE_FBIK_SOLVERS[key] = solver

    import __main__
    if not hasattr(__main__, "_ue58_fbik_solvers"):
        __main__._ue58_fbik_solvers = {}
    __main__._ue58_fbik_solvers[key] = solver

    cb_ids = []
    watched_locators = [info["locator_dag"] for info in solver.goal_locators.values()]
    if solver.pelvis_locator_dag:
        watched_locators.append(solver.pelvis_locator_dag)

    watched_attrs = {
        "translate", "translateX", "translateY", "translateZ",
        "rotate", "rotateX", "rotateY", "rotateZ",
        "positionAlpha", "rotationAlpha",
    }

    def _on_attr_changed(msg, plug, other_plug, client_data):
        if not (msg & om.MNodeMessage.kAttributeSet):
            return
        attr_short = plug.partialName(useLongNames=True)
        if attr_short in watched_attrs:
            solver.solve_from_scene_goals()

    for loc_dag in watched_locators:
        if not cmds.objExists(loc_dag):
            continue
        sel = om.MSelectionList()
        sel.add(loc_dag)
        mobj = sel.getDependNode(0)
        cid = om.MNodeMessage.addAttributeChangedCallback(mobj, _on_attr_changed)
        cb_ids.append(cid)

    _ACTIVE_CALLBACK_IDS[key] = cb_ids

    # Also create a Maya DG Expression node so Goal -> FBIKSolver connections exist in DG
    if not cmds.attributeQuery("fbikEvalOutput", node=fbik_group_dag, exists=True):
        cmds.addAttr(fbik_group_dag, longName="fbikEvalOutput", attributeType="double", keyable=False)

    expr_name = "UE58_FBIK_Expr_{}".format(key)
    if cmds.objExists(expr_name):
        cmds.delete(expr_name)

    terms = []
    for loc_dag in watched_locators:
        if cmds.objExists(loc_dag):
            for attr in ("translateX", "translateY", "translateZ", "rotateX", "rotateY", "rotateZ"):
                terms.append("{}.{}".format(loc_dag, attr))

    if terms:
        sum_expr = " + ".join(terms)
        py_cmd = "import __main__; s = getattr(__main__, '_ue58_fbik_solvers', {{}}).get('{}'); s.solve_from_scene_goals() if s else None".format(key)
        mel_expr = (
            "{fbik_node}.fbikEvalOutput = {sum_expr};\n"
            "python(\"{py_cmd}\");\n"
        ).format(fbik_node=fbik_group_dag, sum_expr=sum_expr, py_cmd=py_cmd)
        try:
            cmds.expression(name=expr_name, string=mel_expr, alwaysEvaluate=False)
        except Exception:
            pass
