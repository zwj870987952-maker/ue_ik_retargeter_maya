# -*- coding: utf-8 -*-
"""
UE5 IK Retargeter for Autodesk Maya.
A complete implementation of Unreal Engine 5's IK Rig and IK Retargeter system
inside Autodesk Maya for high-fidelity cross-skeleton animation retargeting.
Supports one-click hot-reloading without restarting Maya.
"""

import sys
import importlib

SUBMODULES_ORDER = [
    "ue_ik_retargeter_maya.core.math_utils",
    "ue_ik_retargeter_maya.core.skeleton",
    "ue_ik_retargeter_maya.core.chains",
    "ue_ik_retargeter_maya.core.retarget_pose",
    "ue_ik_retargeter_maya.core.pelvis_solver",
    "ue_ik_retargeter_maya.core.fk_solver",
    "ue_ik_retargeter_maya.core.ik_solver",
    "ue_ik_retargeter_maya.core.auto_align",
    "ue_ik_retargeter_maya.core.retarget_cache",
    "ue_ik_retargeter_maya.core.processor",
    "ue_ik_retargeter_maya.rig_driver.driver",
    "ue_ik_retargeter_maya.batch.batch_baker",
    "ue_ik_retargeter_maya.ui.main_window",
]


def reload_plugin():
    """
    Hot-reloads all plugin submodules in dependency order so code edits
    take effect immediately inside a running Maya session.
    """
    importlib.invalidate_caches()

    for mod_name in SUBMODULES_ORDER:
        if mod_name in sys.modules:
            importlib.reload(sys.modules[mod_name])
        else:
            importlib.import_module(mod_name)

    # Refresh top-level symbols after submodule reload
    global IKRetargetProcessor, RetargetSkeleton, BoneChain, ChainMapping
    global RetargetPoseManager, PelvisMotionSolver, FKChainSolver, IKChainSolver
    global RetargetCache, show_ui

    from .core.processor import IKRetargetProcessor
    from .core.skeleton import RetargetSkeleton
    from .core.chains import BoneChain, ChainMapping
    from .core.retarget_pose import RetargetPoseManager
    from .core.pelvis_solver import PelvisMotionSolver
    from .core.fk_solver import FKChainSolver
    from .core.ik_solver import IKChainSolver
    from .core.retarget_cache import RetargetCache
    from .ui.main_window import show_ui

    print("[UE5 IK Retargeter] Hot-reloaded all plugin modules successfully!")
    return True


def launch(hot_reload=True):
    """Hot-reloads the plugin (by default) and opens the UI window."""
    if hot_reload:
        reload_plugin()
    from .ui.main_window import show_ui as _show_ui
    return _show_ui()


from .core.processor import IKRetargetProcessor
from .core.skeleton import RetargetSkeleton
from .core.chains import BoneChain, ChainMapping
from .core.retarget_pose import RetargetPoseManager
from .core.pelvis_solver import PelvisMotionSolver
from .core.fk_solver import FKChainSolver
from .core.ik_solver import IKChainSolver
from .core.retarget_cache import RetargetCache
from .ui.main_window import show_ui

__version__ = "1.1.0"
__all__ = [
    "IKRetargetProcessor",
    "RetargetSkeleton",
    "BoneChain",
    "ChainMapping",
    "RetargetPoseManager",
    "PelvisMotionSolver",
    "FKChainSolver",
    "IKChainSolver",
    "RetargetCache",
    "reload_plugin",
    "launch",
    "show_ui",
]
