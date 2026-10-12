# -*- coding: utf-8 -*-
"""
One-click launcher for Maya IK Retargeter.
Run this script inside Maya's Script Editor (Python tab) or drag and drop into viewport.
"""

import sys
import os

pkg_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else r"c:/Users/zwj87/Documents/逆向/ue_ik_retargeter_maya"
parent_dir = os.path.dirname(pkg_dir)

if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import ue_ik_retargeter_maya
win = ue_ik_retargeter_maya.show_ui()
print("[UE IK Retargeter] Launched successfully in Maya!")
