# -*- coding: utf-8 -*-
"""
UE5 IK Retargeter for Autodesk Maya.
A complete implementation of Unreal Engine 5's IK Rig and IK Retargeter system
inside Autodesk Maya for high-fidelity cross-skeleton animation retargeting.
"""

from .core.processor import IKRetargetProcessor
from .core.skeleton import RetargetSkeleton
from .core.chains import BoneChain, ChainMapping
from .core.retarget_pose import RetargetPoseManager
from .core.pelvis_solver import PelvisMotionSolver
from .core.fk_solver import FKChainSolver
from .core.ik_solver import IKChainSolver
from .ui.main_window import show_ui

__version__ = "1.0.0"
__all__ = [
    "IKRetargetProcessor",
    "RetargetSkeleton",
    "BoneChain",
    "ChainMapping",
    "RetargetPoseManager",
    "PelvisMotionSolver",
    "FKChainSolver",
    "IKChainSolver",
    "show_ui"
]
