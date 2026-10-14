"""Assemble the project's existing FBIK in the open Maya 2024 test scene.

Execute with the Maya MCP execute_python file_path argument. No scene save or
bake is performed. Assembly is a single undo chunk; selection/time are restored.
"""
import json
import sys
import importlib
from pathlib import Path
import maya.cmds as cmds

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
V2 = PROJECT / 'ue58_three_assets_v2'
assert cmds.about(version=True) == '2024', 'Use the Maya 2024 session.'
assert Path(cmds.file(q=True,sn=True)).name == 'test_maya.ma', 'Use test_maya.ma.'
roots = ['|CC_Base_BoneRoot', '|mixamorig:Hips']
assert all(cmds.objExists(r) for r in roots), roots
selection = cmds.ls(sl=True,long=True) or []
current_time = cmds.currentTime(q=True)
baseline = {'scene':cmds.file(q=True,sn=True),'time':current_time,'joints':{},
            'anim_curves':cmds.ls(type='animCurve') or []}
baseline['animation_curve_keys']={n:{'times':cmds.keyframe(n,q=True,timeChange=True),
                                    'values':cmds.keyframe(n,q=True,valueChange=True)}
                                  for n in baseline['anim_curves']}
for root in roots:
    for joint in [root] + (cmds.listRelatives(root,ad=True,type='joint',fullPath=True) or []):
        baseline['joints'][joint] = {attr:list(cmds.getAttr(joint+'.'+attr)[0]) for attr in ['translate','rotate','scale']}
HERE.joinpath('maya_before_assembly.json').write_text(json.dumps(baseline,indent=2),encoding='utf-8')
sys.path.insert(0,str(V2))
# Clear the previous version's callbacks / matrix outputs before unloading any
# modules; otherwise deleting module registries leaves live callbacks behind.
old=sys.modules.get('ue_ik_retargeter_maya.core.fbik_solver')
if old is not None:old.clear_fbik_callbacks()
if 'ue58PBIKSolver' in cmds.allNodeTypes() and not cmds.ls(type='ue58PBIKSolver'):
    cmds.unloadPlugin('ue58_pbik_node')
for name in list(sys.modules):
    if name == 'ue_ik_retargeter_maya' or name.startswith('ue_ik_retargeter_maya.'):
        del sys.modules[name]
importlib.invalidate_caches()
import ue_ik_retargeter_maya as plugin
cmds.undoInfo(openChunk=True,chunkName='UE58 FBIK comparison assembly')
animation_connections=[]
try:
    # xform on connected animation can change existing key values. Isolate the
    # static reference pose during assembly, keeping the curve nodes intact.
    for joint in baseline['joints']:
        for a in ['tx','ty','tz','rx','ry','rz']:
            destination=joint+'.'+a
            for source in cmds.listConnections(destination,s=True,d=False,plugs=True,type='animCurve') or []:
                animation_connections.append((source,destination))
                cmds.disconnectAttr(source,destination)
    # UE asset roles are confirmed from the exported preview meshes before running.
    bundle = plugin.generate_retarget_assets(source_root=roots[0],target_root=roots[1],
        output_dir=str(HERE / 'maya_assets'),build_in_scene=True,
        apply_aligned_pose=False,hot_reload=False)
    import __main__
    __main__.codex_fbik_comparison_bundle = bundle
    summary = {'nodes':bundle['scene_nodes'],'files':bundle['files'],
               'source_profile':bundle['processor'].source_profile['template_name'],
               'target_profile':bundle['processor'].target_profile['template_name']}
    HERE.joinpath('maya_assembly.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print('CODEX_FBIK_ASSEMBLED '+json.dumps(summary))
finally:
    for source,destination in animation_connections:
        cmds.connectAttr(source,destination)
    cmds.currentTime(current_time,e=True)
    cmds.select(selection,r=True) if selection else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
