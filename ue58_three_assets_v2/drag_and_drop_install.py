# -*- coding: utf-8 -*-
"""
One-click Drag-and-Drop Launcher for UE 5.8 IK Retargeter (v2 - Standard 3-Asset Architecture) in Maya.
Usage:
  Method 1 (Drag & Drop): Directly drag `drag_and_drop_install.py` from Windows Explorer into the Maya 3D Viewport!
  Method 2 (Script Editor): Open in Maya Script Editor (Python tab) and press Ctrl+Enter.
"""

import os
import sys
import importlib


def _launch_retargeter():
    if "__file__" in globals():
        v2_root = os.path.dirname(os.path.abspath(__file__))
    else:
        v2_root = r"c:/Users/zwj87/Documents/逆向/ue_ik_retargeter_maya/ue58_three_assets_v2"

    # Preserve live DG drivers on reload and clear them before switching folders.
    package = sys.modules.get("ue_ik_retargeter_maya")
    loaded_root = os.path.dirname(getattr(package, "__file__", ""))
    expected_root = os.path.join(v2_root, "ue_ik_retargeter_maya")
    if package and os.path.normcase(os.path.abspath(loaded_root)) != os.path.normcase(os.path.abspath(expected_root)):
        old_solver = sys.modules.get("ue_ik_retargeter_maya.core.fbik_solver")
        if old_solver and hasattr(old_solver, "clear_fbik_callbacks"):
            old_solver.clear_fbik_callbacks()
        for mod_name in list(sys.modules):
            if mod_name == "ue_ik_retargeter_maya" or mod_name.startswith("ue_ik_retargeter_maya."):
                del sys.modules[mod_name]

    if v2_root in sys.path:
        sys.path.remove(v2_root)
    sys.path.insert(0, v2_root)
    importlib.invalidate_caches()

    import ue_ik_retargeter_maya
    importlib.reload(ue_ik_retargeter_maya)
    win = ue_ik_retargeter_maya.launch(hot_reload=True)
    print("[UE5.8 IK Retargeter v2] Hot-reloaded and launched successfully from: {}".format(v2_root))
    return win


def onMayaDroppedPythonFile(*args, **kwargs):
    """Triggered automatically when dragged from Windows Explorer into the Maya 3D Viewport."""
    _launch_retargeter()


if __name__ == "__main__":
    _launch_retargeter()
