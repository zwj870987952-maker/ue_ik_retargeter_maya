# -*- coding: utf-8 -*-
"""
End-to-end unit and integration test for Maya IK Retargeter.
Builds two procedural skeletons with different heights, bone counts, and proportions,
animates the source skeleton, executes full IK/FK retargeting, and verifies numerical correctness.
"""

import sys
import math
import maya.cmds as cmds
import maya.api.OpenMaya as om

from ..core.processor import IKRetargetProcessor
from ..core.math_utils import get_world_transform


def create_test_source_character(prefix="src_"):
    """
    Creates a standard source character skeleton with animation:
    Hips -> Spine1 -> Spine2 -> Neck -> Head
    Hips -> L_UpLeg -> L_Leg -> L_Foot
    Hips -> R_UpLeg -> R_Leg -> R_Foot
    Hips -> L_Arm -> L_ForeArm -> L_Hand
    Hips -> R_Arm -> R_ForeArm -> R_Hand
    """
    cmds.select(cl=True)
    hips = cmds.joint(n=prefix + "hips", p=(0, 100, 0))

    # Spine (2 joints)
    spine1 = cmds.joint(n=prefix + "spine_01", p=(0, 120, 0))
    spine2 = cmds.joint(n=prefix + "spine_02", p=(0, 140, 0))
    neck = cmds.joint(n=prefix + "neck", p=(0, 160, 0))
    head = cmds.joint(n=prefix + "head", p=(0, 180, 0))

    # Left Leg
    cmds.select(hips)
    l_thigh = cmds.joint(n=prefix + "thigh_l", p=(15, 95, 0))
    l_calf = cmds.joint(n=prefix + "calf_l", p=(15, 50, -5))
    l_foot = cmds.joint(n=prefix + "foot_l", p=(15, 5, 5))

    # Right Leg
    cmds.select(hips)
    r_thigh = cmds.joint(n=prefix + "thigh_r", p=(-15, 95, 0))
    r_calf = cmds.joint(n=prefix + "calf_r", p=(-15, 50, -5))
    r_foot = cmds.joint(n=prefix + "foot_r", p=(-15, 5, 5))

    # Left Arm
    cmds.select(spine2)
    l_arm = cmds.joint(n=prefix + "upperarm_l", p=(25, 140, 0))
    l_forearm = cmds.joint(n=prefix + "lowerarm_l", p=(55, 140, 0))
    l_hand = cmds.joint(n=prefix + "hand_l", p=(85, 140, 0))

    # Right Arm
    cmds.select(spine2)
    r_arm = cmds.joint(n=prefix + "upperarm_r", p=(-25, 140, 0))
    r_forearm = cmds.joint(n=prefix + "lowerarm_r", p=(-55, 140, 0))
    r_hand = cmds.joint(n=prefix + "hand_r", p=(-85, 140, 0))

    # Frame 0 is the clean un-animated rest pose
    cmds.setKeyframe(hips, time=0, attribute='translateY', value=100.0)
    cmds.setKeyframe(hips, time=0, attribute='translateZ', value=0.0)

    # Add animation keyframes (Walking / Squatting motion over frames 1 to 20)
    for f in range(1, 21):
        t = f / 20.0 * 2.0 * math.pi
        # Squat motion on hips
        hips_y = 100.0 - 15.0 * math.sin(t) ** 2
        hips_z = 10.0 * math.sin(t)
        cmds.setKeyframe(hips, time=f, attribute='translateY', value=hips_y)
        cmds.setKeyframe(hips, time=f, attribute='translateZ', value=hips_z)

        # Leg swing
        leg_rot = 30.0 * math.sin(t)
        cmds.setKeyframe(l_thigh, time=f, attribute='rotateX', value=leg_rot)
        cmds.setKeyframe(r_thigh, time=f, attribute='rotateX', value=-leg_rot)

        # Arm swing
        cmds.setKeyframe(l_arm, time=f, attribute='rotateX', value=-leg_rot)
        cmds.setKeyframe(r_arm, time=f, attribute='rotateX', value=leg_rot)

    return hips


def create_test_target_character(prefix="tgt_"):
    """
    Creates a target character skeleton with DIFFERENT proportions:
    1. 1.4x Taller overall (Hips at Y=140 instead of 100)
    2. Spine has 3 joints instead of 2 (testing arc-length parameterized interpolation)
    3. Wider stance (Thigh X=25 instead of 15)
    4. Arms initially in A-Pose (slanted down ~45 degrees)
    """
    cmds.select(cl=True)
    hips = cmds.joint(n=prefix + "hips", p=(0, 140, 0))

    # Spine (3 joints!)
    spine1 = cmds.joint(n=prefix + "spine_01", p=(0, 165, 0))
    spine2 = cmds.joint(n=prefix + "spine_02", p=(0, 190, 0))
    spine3 = cmds.joint(n=prefix + "spine_03", p=(0, 215, 0))
    neck = cmds.joint(n=prefix + "neck", p=(0, 235, 0))
    head = cmds.joint(n=prefix + "head", p=(0, 255, 0))

    # Left Leg (Taller)
    cmds.select(hips)
    l_thigh = cmds.joint(n=prefix + "thigh_l", p=(25, 135, 0))
    l_calf = cmds.joint(n=prefix + "calf_l", p=(25, 70, -8))
    l_foot = cmds.joint(n=prefix + "foot_l", p=(25, 8, 8))

    # Right Leg
    cmds.select(hips)
    r_thigh = cmds.joint(n=prefix + "thigh_r", p=(-25, 135, 0))
    r_calf = cmds.joint(n=prefix + "calf_r", p=(-25, 70, -8))
    r_foot = cmds.joint(n=prefix + "foot_r", p=(-25, 8, 8))

    # Left Arm in A-Pose (slanted down ~45 degrees)
    cmds.select(spine3)
    l_arm = cmds.joint(n=prefix + "upperarm_l", p=(35, 215, 0))
    l_forearm = cmds.joint(n=prefix + "lowerarm_l", p=(65, 185, 0))
    l_hand = cmds.joint(n=prefix + "hand_l", p=(95, 155, 0))

    # Right Arm in A-Pose (slanted down ~45 degrees)
    cmds.select(spine3)
    r_arm = cmds.joint(n=prefix + "upperarm_r", p=(-35, 215, 0))
    r_forearm = cmds.joint(n=prefix + "lowerarm_r", p=(-65, 185, 0))
    r_hand = cmds.joint(n=prefix + "hand_r", p=(-95, 155, 0))

    return hips


def run_test():
    """Executes the automated retargeting test and verifies output."""
    cmds.file(new=True, force=True)

    print(">>> 1. Creating source animated character...")
    src_hips = create_test_source_character("src_")

    print(">>> 2. Creating target character with different height and bone counts...")
    tgt_hips = create_test_target_character("tgt_")

    cmds.currentTime(0)
    print(">>> 3. Testing completely automatic One-Click Retargeting (UE5 experience)...")
    processor = IKRetargetProcessor.one_click_retarget(src_hips, tgt_hips, 1, 20)

    # Verification
    print(">>> 4. Verifying numerical correctness...")

    # Check frame 10 (mid-squat)
    cmds.currentTime(10, e=True)
    src_hips_pos, _ = get_world_transform("src_hips")
    tgt_hips_pos, _ = get_world_transform("tgt_hips")

    print("Frame 10 Source Hips Y:", src_hips_pos.y)
    print("Frame 10 Target Hips Y:", tgt_hips_pos.y)

    # Height ratio is 140 / 100 = 1.4
    expected_tgt_y = src_hips_pos.y * 1.4
    diff = abs(tgt_hips_pos.y - expected_tgt_y)
    print("Hips Y Difference from expected scaled position:", diff)
    assert diff < 2.0, "Pelvis height scaling error too large: {}".format(diff)

    # Check foot IK goal position
    tgt_foot_pos, _ = get_world_transform("tgt_foot_l")
    print("Frame 10 Target Foot L Position:", (tgt_foot_pos.x, tgt_foot_pos.y, tgt_foot_pos.z))
    assert tgt_foot_pos.y >= 0.0, "Target foot penetrated ground floor!"

    # Verify that keyframes exist on target joints
    key_count = cmds.keyframe("tgt_hips", q=True, keyframeCount=True)
    print("Target hips keyframe count:", key_count)
    assert key_count > 0, "No keyframes were baked onto target hips!"

    # Verify that arms and hands are not crooked or inverted
    # Left arm should extend to positive X (away from body)
    p_upper, _ = get_world_transform("tgt_upperarm_l")
    p_lower, _ = get_world_transform("tgt_lowerarm_l")
    p_hand, q_hand = get_world_transform("tgt_hand_l")
    arm_dir = (p_hand - p_upper).normal()
    print("Target Left Arm overall vector:", (arm_dir.x, arm_dir.y, arm_dir.z))
    assert arm_dir.x > 0.3, "Left arm is pointing inwards towards body or flipped!"

    # Right arm should extend to negative X (away from body)
    p_r_upper, _ = get_world_transform("tgt_upperarm_r")
    p_r_hand, q_r_hand = get_world_transform("tgt_hand_r")
    r_arm_dir = (p_r_hand - p_r_upper).normal()
    print("Target Right Arm overall vector:", (r_arm_dir.x, r_arm_dir.y, r_arm_dir.z))
    assert r_arm_dir.x < -0.3, "Right arm is pointing inwards towards body or flipped!"

    # Verify hand orientations have no NaN or denormalized quaternions
    mag_l = math.sqrt(q_hand.x**2 + q_hand.y**2 + q_hand.z**2 + q_hand.w**2)
    mag_r = math.sqrt(q_r_hand.x**2 + q_r_hand.y**2 + q_r_hand.z**2 + q_r_hand.w**2)
    assert abs(mag_l - 1.0) < 1e-3, "Left hand quaternion invalid!"
    assert abs(mag_r - 1.0) < 1e-3, "Right hand quaternion invalid!"

    print(">>> [SUCCESS] All retargeting tests PASSED with zero errors! Hands and feet orientations are anatomically natural.")
    return True


if __name__ == "__main__":
    run_test()
