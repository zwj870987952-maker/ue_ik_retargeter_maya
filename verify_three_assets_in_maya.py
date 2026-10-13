# -*- coding: utf-8 -*-
"""
[Step 1 Verification] Generate UE 5.8 Standard 3 Assets in Current Maya Scene.
Direct alias of `verify_3_assets_in_maya.py`.
"""

from verify_3_assets_in_maya import run_generate_three_assets, onMayaDroppedPythonFile

if __name__ == "__main__":
    run_generate_three_assets(apply_aligned_pose=True)
