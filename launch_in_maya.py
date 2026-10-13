# -*- coding: utf-8 -*-
"""
One-click Drag-and-Drop Launcher for UE 5.8 IK Retargeter in Maya.
Usage:
  Method 1 (Drag & Drop): Directly drag `launch_in_maya.py` (or `drag_and_drop_install.py`)
                          from Windows Explorer into the Maya 3D Viewport!
  Method 2 (Script Editor): Open in Maya Script Editor (Python tab) and press Ctrl+Enter.
Automatically hot-reloads all plugin code and opens the UI on top of Maya.
"""

import os
import sys
import importlib


def _resolve_package_parent():
    if "__file__" in globals():
        this_dir = os.path.dirname(os.path.abspath(__file__))
        v2_candidate = os.path.join(this_dir, "ue58_three_assets_v2")
        if os.path.isdir(os.path.join(v2_candidate, "ue_ik_retargeter_maya")):
            return v2_candidate
        if os.path.isdir(os.path.join(this_dir, "ue_ik_retargeter_maya")):
            return this_dir
        if os.path.basename(this_dir) == "ue_ik_retargeter_maya":
            return os.path.dirname(this_dir)
        return this_dir
    return r"c:/Users/zwj87/Documents/逆向/ue_ik_retargeter_maya/ue58_three_assets_v2"


def _launch_retargeter():
    """Resolves package path, hot-reloads all submodules, and opens the Retargeter UI."""
    pkg_parent = _resolve_package_parent()

    for mod_name in list(sys.modules.keys()):
        if mod_name == "ue_ik_retargeter_maya" or mod_name.startswith("ue_ik_retargeter_maya."):
            del sys.modules[mod_name]

    if pkg_parent in sys.path:
        sys.path.remove(pkg_parent)
    sys.path.insert(0, pkg_parent)
    importlib.invalidate_caches()

    import ue_ik_retargeter_maya
    win = ue_ik_retargeter_maya.launch(hot_reload=True)
    print("[UE5.8 IK Retargeter] Hot-reloaded and launched successfully from: {}".format(pkg_parent))
    return win


def onMayaDroppedPythonFile(*args, **kwargs):
    """
    Official Maya 2022/2024/2025 callback triggered automatically when this .py file
    is dragged and dropped from Windows Explorer directly into the Maya 3D Viewport.
    """
    _launch_retargeter()


if __name__ == "__main__":
    _launch_retargeter()
