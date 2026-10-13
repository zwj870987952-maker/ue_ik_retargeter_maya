# -*- coding: utf-8 -*-
"""
One-click Drag-and-Drop Launcher for UE5 IK Retargeter in Maya.
Usage:
  Method 1 (Drag & Drop): Directly drag `launch_in_maya.py` from Windows Explorer into the Maya 3D Viewport!
  Method 2 (Script Editor): Open in Maya Script Editor (Python tab) and press Ctrl+Enter.
Automatically hot-reloads all plugin code and opens the UI on top of Maya.
"""

import sys
import os


def _launch_retargeter():
    """Resolves package path, hot-reloads all submodules, and opens the Retargeter UI."""
    if '__file__' in globals():
        this_dir = os.path.dirname(os.path.abspath(__file__))
        # Check if this script is inside `ue_ik_retargeter_maya` or in the workspace root
        if os.path.basename(this_dir) == "ue_ik_retargeter_maya":
            parent_dir = os.path.dirname(this_dir)
        else:
            parent_dir = this_dir
    else:
        parent_dir = r"c:/Users/zwj87/Documents/逆向"

    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)

    import importlib
    import ue_ik_retargeter_maya
    importlib.reload(ue_ik_retargeter_maya)
    win = ue_ik_retargeter_maya.launch(hot_reload=True)
    print("[UE5 IK Retargeter] Hot-reloaded and launched successfully in Maya!")
    return win


def onMayaDroppedPythonFile(*args, **kwargs):
    """
    Official Maya 2022/2024/2025 callback triggered automatically when this .py file
    is dragged and dropped from Windows Explorer directly into the Maya 3D Viewport.
    """
    _launch_retargeter()


if __name__ == "__main__":
    # Triggered when executed directly inside Maya's Script Editor
    _launch_retargeter()
