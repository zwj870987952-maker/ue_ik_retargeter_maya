"""Verify the shopping session, delete its systems, and rebuild them from cache."""
import hashlib
import json
import math
import __main__
from pathlib import Path
import maya.cmds as cmds
import maya.api.OpenMaya as om
from ue_ik_retargeter_maya.core.cache_workflow import (
    _cached_joints,_original_joints,scene_state,remove_cached_systems,retarget_from_cache)

HERE=Path(__file__).parent
session=__main__._ue58_cache_session
manifest=__main__._ue58_cache_build['manifest']
original={r:_original_joints(session[r.lower()+'_root'],manifest['records'][r]) for r in ['Source','Target']}
cached={r:_cached_joints(session['systems'][r]) for r in original}
frames=[0,30,60,90,120,160,200,240,300,360,403,120,0]
report={'clip':'shop_sales_walkintro_f.fbx','range':[0,403],'retained':True}

def curve_hash(joints):
    curves=sorted(set(cmds.listConnections(joints,s=True,d=False,type='animCurve') or []))
    entries=[(n,cmds.keyframe(n,q=True,timeChange=True),cmds.keyframe(n,q=True,valueChange=True)) for n in curves]
    return {'curves':len(curves),'hash':hashlib.sha256(json.dumps(entries).encode()).hexdigest()}

def poses(joints,display=None):
    output={}
    with scene_state(temporary=True):
        for frame in frames:
            cmds.currentTime(frame,e=True)
            inverse=om.MMatrix(cmds.xform(display,q=True,ws=True,matrix=True)).inverse() if display else om.MMatrix()
            output[frame]=[list(om.MMatrix(cmds.xform(j,q=True,ws=True,matrix=True))*inverse) for j in joints]
    return output

def error(a,b):
    return max(abs(x-y) for frame in a for ma,mb in zip(a[frame],b[frame]) for x,y in zip(ma,mb))

before_source=curve_hash(original['Source']);before_target=curve_hash(original['Target'])
before_pose=poses(original['Target'])
report['source_copy_matrix_error']=error(poses(original['Source']),poses(cached['Source'],session['systems']['Source']))
report['bake_matrix_error']=error(before_pose,poses(cached['Target'],session['systems']['Target']))
assert report['source_copy_matrix_error']<1e-6
assert report['bake_matrix_error']<1e-6
report['target_opm_connections']=sum(bool(cmds.listConnections(j+'.offsetParentMatrix',s=True,d=False)) for j in original['Target'])
assert report['target_opm_connections']==0

config_text=cmds.getAttr(session['target_node']+'.configuration')
config=json.loads(config_text)
for op in config['retarget_bundle']['rtg']['retarget_ops']:
    if op['op_type'] in ['FIKRetargetIKChainsOp','FIKRetargetRunIKRigOp']:op['enabled']=True
with scene_state(temporary=True):
    try:
        cmds.setAttr(session['target_node']+'.configuration',json.dumps(config),type='string')
        solves=[]
        for frame in [0,60,120,240,403]:
            cmds.currentTime(frame,e=True)
            matrices=[cmds.xform(j,q=True,ws=True,matrix=True) for j in cached['Target']]
            assert all(math.isfinite(v) for m in matrices for v in m)
            sel=om.MSelectionList();sel.add(session['target_node'])
            runtime=om.MFnDependencyNode(sel.getDependNode(0)).userNode().retarget_runtime
            solves.append(runtime.last_native_solves)
        assert solves==[1]*5
        report['optional_native_solves_per_evaluation']=solves
    finally:cmds.setAttr(session['target_node']+'.configuration',config_text,type='string')

# A locked output must fail before changing original animation, and the file
# import must be cleaned even though Maya's import is not completely undoable.
namespaces_before=set(cmds.namespaceInfo(':',listOnlyNamespaces=True,recurse=True) or [])
plug=original['Target'][0]+'.rotateX';locked=cmds.getAttr(plug,lock=True)
try:
    cmds.setAttr(plug,lock=True)
    try:
        retarget_from_cache(__main__._ue58_cache_build['rtg'],session['source_root'],session['target_root'],0,1,True)
        raise AssertionError('Locked output unexpectedly accepted')
    except ValueError as exc:
        assert 'Locked target channel' in str(exc)
        report['locked_output_rejected']=True
finally:cmds.setAttr(plug,lock=locked)
report['failed_import_namespaces_preserved']=namespaces_before==set(cmds.namespaceInfo(':',listOnlyNamespaces=True,recurse=True) or [])
assert report['failed_import_namespaces_preserved']
assert curve_hash(original['Source'])==before_source and curve_hash(original['Target'])==before_target

remove_cached_systems(session['session_root'])
try:
    assert not cmds.objExists(session['session_root'])
    report['after_system_deletion_matrix_error']=error(before_pose,poses(original['Target']))
    assert report['after_system_deletion_matrix_error']==0.
    assert curve_hash(original['Source'])==before_source and curve_hash(original['Target'])==before_target
finally:
    # A fresh import also exercises repeatability, and does not depend on the
    # outer undo chunk the MCP host uses to execute this script.
    session=retarget_from_cache(__main__._ue58_cache_build['rtg'],session['source_root'],session['target_root'],0,403,True)
    __main__._ue58_cache_session=session
    cached={r:_cached_joints(session['systems'][r]) for r in original}
assert cmds.objExists(session['session_root'])
report['restored_systems_matrix_error']=error(before_pose,poses(cached['Target'],session['systems']['Target']))
assert report['restored_systems_matrix_error']<1e-6
report['source_animation']=before_source;report['target_animation']=before_target
report['passed']=True
(HERE/'live_validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
