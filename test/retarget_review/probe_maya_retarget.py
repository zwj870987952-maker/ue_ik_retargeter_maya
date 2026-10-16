"""Probe retargeting on isolated copies of the open scene's two skeletons.

Creates synthetic motion only in the test namespace. Never previews or bakes
onto the user's source / target joints; always removes the test namespace.
"""
import copy
import importlib
import json
import math
import traceback
from pathlib import Path
import maya.cmds as cmds
import maya.api.OpenMaya as om
from ue_ik_retargeter_maya.core.math_utils import get_world_transform
from ue_ik_retargeter_maya.core import processor as processor_module
from ue_ik_retargeter_maya.core import retarget_assets as assets
from ue_ik_retargeter_maya.core import fbik_scene
from ue_ik_retargeter_maya.core.pbik_native import NativePBIKSolver

namespace='__UE58RetargetReview'
assert not cmds.namespace(exists=namespace)
processor_module=importlib.reload(processor_module)
fbik_scene=importlib.reload(fbik_scene)
old_builder_class=assets.FullBodyIKSolver
assets.FullBodyIKSolver=fbik_scene.FullBodyIKSolver
frame=cmds.currentTime(q=True);selection=cmds.ls(sl=True,long=True) or []
current_namespace=cmds.namespaceInfo(currentNamespace=True)
original_nodes=cmds.ls(type='ue58PBIKSolver')
original_joints=sorted({plug.rsplit('.matrix',1)[0] for n in original_nodes
    for plug in (cmds.listConnections(n+'.inputLocalMatrix',s=True,d=False,p=True) or [])})
def fingerprint():
    curves={n:{'times':cmds.keyframe(n,q=True,timeChange=True),'values':cmds.keyframe(n,q=True,valueChange=True),
                'destinations':cmds.listConnections(n,s=False,d=True,p=True)} for n in cmds.ls(type='animCurve')}
    return {'curves':curves,'joints':{j:{a:cmds.getAttr(j+'.'+a) for a in ['translate','rotate','scale']} for j in original_joints}}
before=fingerprint();registry_before=set(fbik_scene._ACTIVE_FBIK_SOLVERS)
report={'scene':cmds.file(q=True,sn=True),'original_frame':frame,'original_curves':len(before['curves']),
        'varying_original_curves':sum(len(set(v['values'] or []))>1 for v in before['curves'].values()),
        'auto_key_enabled':cmds.autoKeyframe(q=True,state=True),'cases':{},'errors':[]}
solver=None;key='RetargetReviewTarget'
original_write=processor_module.set_world_transform
original_native=NativePBIKSolver.solve
original_two_bone=processor_module.TwoBoneIKSolver.solve
counts={'native':0,'two_bone':0};expected={}
def native_spy(self,*args,**kwargs):
    counts['native']+=1
    return original_native(self,*args,**kwargs)
def two_bone_spy(*args,**kwargs):
    counts['two_bone']+=1
    return original_two_bone(*args,**kwargs)
def write_spy(joint,pos=None,quat=None):
    expected[joint]=(om.MVector(pos) if pos is not None else None,om.MQuaternion(quat) if quat is not None else None)
    return original_write(joint,pos,quat)
def rotation_error(a,b):
    return math.degrees(2*math.acos(min(1.,abs(sum(x*y for x,y in zip(a,b))))))
def run_case(proc):
    counts.update(native=0,two_bone=0);expected.clear()
    proc.retarget_frame()
    actual={j:get_world_transform(j) for j in proc.target_skeleton.joint_names}
    return {'calls':dict(counts),'maximum_rotation_difference_degrees':max(rotation_error(q,actual[j][1]) for j,(_,q) in expected.items() if q is not None),
            'pelvis_position_difference_cm':(expected[proc.target_pelvis][0]-actual[proc.target_pelvis][0]).length()}

cmds.undoInfo(openChunk=True,chunkName='UE58 isolated retarget workflow audit')
try:
    cmds.namespace(add=namespace)
    owner=cmds.createNode('transform',name=':'+namespace+':Sandbox',skipSelect=True)
    cmds.setAttr(owner+'.visibility',False)
    roots={}
    for role,node in [('Src',next(n for n in original_nodes if '_Src_' in n)),('Tgt',next(n for n in original_nodes if '_Tgt_' in n))]:
        cmds.namespace(add=role,parent=namespace)
        cfg=json.loads(cmds.getAttr(node+'.configuration'))
        original=[]
        for index in range(len(cfg['parents'])):
            plug=cmds.listConnections(node+'.inputLocalMatrix[{}]'.format(index),s=True,d=False,p=True)[0]
            original.append(plug.rsplit('.matrix',1)[0])
        external=cmds.createNode('transform',name=':'+namespace+':'+role+':Placement',parent=owner,skipSelect=True)
        parents=cmds.listRelatives(original[0],parent=True,fullPath=True) or []
        if parents:cmds.xform(external,ws=True,matrix=cmds.xform(parents[0],q=True,ws=True,matrix=True))
        duplicates=[]
        for index,joint in enumerate(original):
            parent=duplicates[cfg['parents'][index]] if cfg['parents'][index]>=0 else external
            duplicate=cmds.createNode('joint',name=':'+namespace+':'+role+':'+joint.split('|')[-1].split(':')[-1],parent=parent,skipSelect=True)
            for attr in ['rotateOrder','segmentScaleCompensate']:
                cmds.setAttr(duplicate+'.'+attr,cmds.getAttr(joint+'.'+attr))
            for attr in ['jointOrient','rotateAxis','translate','rotate','scale','inverseScale']:
                cmds.setAttr(duplicate+'.'+attr,*cmds.getAttr(joint+'.'+attr)[0])
            cmds.setAttr(duplicate+'.offsetParentMatrix',*cfg['baseline_offsets'][index],type='matrix')
            duplicates.append(cmds.ls(duplicate,long=True)[0])
        roots[role]=duplicates[0]
    proc=processor_module.IKRetargetProcessor(roots['Src'],roots['Tgt'])
    report['profiles']=[proc.source_profile['template_name'],proc.target_profile['template_name']]
    report['mapped_chains']=len(proc.chain_mappings)
    source_cache=proc.source_ikrig_asset.to_dict()
    target_cache=proc.target_ikrig_asset.to_dict()
    mapping=next(m for m in proc.chain_mappings if m.name=='LeftArm')
    data=proc.rtg_asset.to_dict();changed=copy.deepcopy(data)
    entry=next(m for m in changed['chain_mappings'] if m['target_chain']=='LeftArm')
    entry.update(source_chain='RightArm',rotation_alpha=.123,enable_fk=False)
    assets.ThreeAssetLoader.apply_three_asset_dicts(proc,None,None,changed)
    report['serialized_mapping_reload']={'requested_source':'RightArm','actual_source':mapping.source_chain.name,
        'requested_alpha':.123,'actual_alpha':mapping.rotation_alpha,'requested_enable_fk':False,'actual_enable_fk':mapping.enable_fk}
    assets.ThreeAssetLoader.apply_three_asset_dicts(proc,None,None,data)
    NativePBIKSolver.solve=native_spy
    processor_module.TwoBoneIKSolver.solve=staticmethod(two_bone_spy)
    processor_module.set_world_transform=write_spy
    report['cases']['reference_without_dg']=run_case(proc)
    source_arm=proc.source_skeleton.resolve_bone_dag('L_Upperarm')
    cmds.setAttr(source_arm+'.rx',cmds.getAttr(source_arm+'.rx')+25.)
    cmds.setAttr(proc.source_pelvis+'.ty',cmds.getAttr(proc.source_pelvis+'.ty')-10.)
    report['cases']['synthetic_motion_without_dg']=run_case(proc)
    target_arm=proc.target_skeleton.resolve_bone_dag('LeftArm')
    for t in [0.,1.]:cmds.setKeyframe(target_arm,attribute='rotateX',time=t,value=0.)
    curve=cmds.listConnections(target_arm+'.rotateX',s=True,d=False,type='animCurve')[0]
    curve_before=cmds.keyframe(curve,q=True,valueChange=True)
    report['cases']['preview_on_animated_target']=run_case(proc)
    curve_after=cmds.keyframe(curve,q=True,valueChange=True)
    report['target_keys_during_single_frame_preview']={'before':curve_before,'after':curve_after,'changed':curve_before!=curve_after}
    cmds.delete(curve)
    # Use the production scene builder for the interaction with the new DG node.
    cmds.namespace(set=namespace)
    asset=assets.IKRigAsset('IK_RetargetReviewTarget','Target',proc.target_skeleton,proc.target_profile)
    _,solver=assets.MayaSceneAssetBuilder._build_ikrig_scene_node(owner,asset,key,proc.target_pose_mgr,'',(.9,.8,.1),(.9,.8,.1))
    proc.target_fbik_solver=solver
    report['goal_modes']=[cmds.getAttr(g['locator_dag']+'.previewMode') for g in solver.goal_locators.values()]
    report['cases']['synthetic_motion_with_dg']=run_case(proc)
    report['cases']['repeat_same_frame_with_dg']=run_case(proc)
    proc.bake_animation(int(frame),int(frame))
    baked={j:get_world_transform(j) for j in proc.target_skeleton.joint_names}
    cmds.setAttr(solver.settings_group+'.enabled',False)
    unassisted={j:get_world_transform(j) for j in proc.target_skeleton.joint_names}
    report['bake_with_dg_then_disable']={'maximum_position_difference_cm':max((baked[j][0]-unassisted[j][0]).length() for j in baked),
        'maximum_rotation_difference_degrees':max(rotation_error(baked[j][1],unassisted[j][1]) for j in baked)}
    # _ensure_processor reapplies this same cache before every preview click.
    # Test whether scene placement is mistakenly inferred from animated Hips.
    proc.target_fbik_solver=None
    root_entry=next(b for b in target_cache['skeleton']['bones'] if b['dag_path']==proc.target_root)
    original_write(proc.target_root,om.MVector(*root_entry['ref_world_pos']),om.MQuaternion(*root_entry['ref_world_rot']))
    cmds.setAttr(proc.source_pelvis+'.tx',cmds.getAttr(proc.source_pelvis+'.tx')+15.)
    cache_samples=[]
    for _ in range(3):
        assets.ThreeAssetLoader.apply_three_asset_dicts(proc,source_cache,target_cache,data)
        run_case(proc)
        cache_samples.append(list(get_world_transform(proc.target_pelvis)[0]))
    report['repeated_cached_preview_at_same_frame']={'pelvis_positions':cache_samples,
        'first_to_third_position_difference_cm':(om.MVector(*cache_samples[2])-om.MVector(*cache_samples[0])).length()}
except Exception:
    report['errors'].append(traceback.format_exc())
finally:
    NativePBIKSolver.solve=original_native
    processor_module.TwoBoneIKSolver.solve=staticmethod(original_two_bone)
    processor_module.set_world_transform=original_write
    assets.FullBodyIKSolver=old_builder_class
    fbik_scene.clear_fbik_callbacks(key)
    cmds.namespace(set=':')
    nodes=cmds.ls(namespace+':*',recursive=True,long=True) or []
    if nodes:cmds.delete(nodes)
    for suffix in [':Src',':Tgt','']:
        if cmds.namespace(exists=namespace+suffix):cmds.namespace(removeNamespace=namespace+suffix)
    cmds.namespace(set=current_namespace)
    cmds.currentTime(frame,e=True)
    cmds.select(selection,r=True) if selection else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
report['original_animation_and_channels_preserved']=before==fingerprint()
report['original_solver_registry_preserved']=registry_before==set(fbik_scene._ACTIVE_FBIK_SOLVERS)
report['test_namespace_removed']=not cmds.namespace(exists=namespace)
Path(__file__).with_name('maya_retarget_review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
