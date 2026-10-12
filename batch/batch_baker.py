# -*- coding: utf-8 -*-
"""
Batch animation baker.
Duplicates and retargets multiple FBX files from a folder onto the target skeleton.
Parallels UE5's FIKRetargetBatchOperationContext (IKRetargetBatchOperation.cpp).
"""

import os
import maya.cmds as cmds
from ..core.processor import IKRetargetProcessor


class BatchRetargeter(object):
    """
    Processes multiple FBX motion files sequentially, retargeting each to the target mesh.
    """

    def __init__(self, processor, output_dir=None):
        self.processor = processor
        self.output_dir = output_dir

    def retarget_fbx(self, fbx_file, export_path=None):
        """Imports an FBX file, retargets the animation onto the target, and exports."""
        if not os.path.exists(fbx_file):
            return False

        # Import FBX onto the source skeleton (namespace merging)
        cmds.file(fbx_file, i=True, type='FBX', mergeNamespacesOnClash=True, namespace=':')

        start_f = int(cmds.playbackOptions(q=True, minTime=True))
        end_f = int(cmds.playbackOptions(q=True, maxTime=True))

        # Bake retargeted animation
        self.processor.bake_animation(start_f, end_f)

        if export_path:
            # Select target skeleton root
            cmds.select(self.processor.target_skeleton.root_joint, r=True)
            cmds.file(
                export_path,
                force=True,
                options='v=0;',
                type='FBX export',
                preserveReferences=True,
                exportSelected=True
            )

        return True

    def process_directory(self, input_dir, output_dir=None):
        """Processes all FBX files in a directory."""
        out_dir = output_dir or self.output_dir or input_dir
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        fbx_files = [
            os.path.join(input_dir, f) for f in os.listdir(input_dir) if f.lower().endswith('.fbx')
        ]

        results = []
        for fbx in fbx_files:
            base_name = os.path.splitext(os.path.basename(fbx))[0]
            out_file = os.path.join(out_dir, base_name + '_retargeted.fbx')
            success = self.retarget_fbx(fbx, out_file)
            results.append((fbx, success))

        return results
