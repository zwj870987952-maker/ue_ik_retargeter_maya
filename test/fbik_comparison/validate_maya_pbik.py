"""Reversible live-Maya acceptance checks; preserves user animation and pose."""
import json
import math
from pathlib import Path
import maya.cmds as cmds
import maya.api.OpenMaya as om
from ue_ik_retargeter_maya.core.fbik_scene import _ACTIVE_FBIK_SOLVERS
from ue_ik_retargeter_maya.core.math_utils import get_world_transform, set_world_transform, slerp, matrix_to_trs, trs_to_matrix
from ue_ik_retargeter_maya.core.pbik_native import to_engine_pose

HERE = Path(__file__).resolve().parent

def angle(a,b):
    q=(a.inverse()*b).normal()
    return math.degrees(2*math.acos(max(-1.,min(1.,abs(q.w)))))

def animation():
    return {n:{'times':cmds.keyframe(n,q=True,timeChange=True),
               'values':cmds.keyframe(n,q=True,valueChange=True),
               'destinations':sorted(cmds.listConnections(n+'.output',s=False,d=True,p=True) or [])}
            for n in cmds.ls(type='animCurve') or []}

def incoming_pose(solver):
    # A disabled node exposes raw animation. Additive goals are relative to the
    # incoming pose AFTER retarget-pose alignment when that option is enabled.
    cfg=json.loads(cmds.getAttr(solver.fbik_node+'.configuration'))
    external=om.MMatrix(cmds.getAttr(solver.fbik_node+'.externalParentMatrix'))
    world=[];pose={}
    for i,n in enumerate(solver.skeleton.joint_names):
        local=om.MMatrix(cmds.getAttr(n+'.matrix'))*om.MMatrix(cfg['baseline_offsets'][i])
        delta=cfg['alignment_rotations'][i]
        if delta is not None:
            p,q,scale=matrix_to_trs(local)
            local=trs_to_matrix(p,(q*om.MQuaternion(*delta)).normal(),scale)
        parent=cfg['parents'][i]
        matrix=local*(world[parent] if parent>=0 else external);world.append(matrix)
        p,q,_=matrix_to_trs(matrix);pose[n]=(p,q)
    return pose

def validate_rig(solver):
    controls=[i['locator_dag'] for i in solver.goal_locators.values()]
    channels={n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate','scale']} for n in controls}
    weights={n:{a:cmds.getAttr(n+'.'+a) for a in ['positionAlpha','rotationAlpha','previewMode']}
             for n in controls}
    stiff={n:cmds.getAttr(n+'.rotationStiffness') for n in solver.bone_setting_nodes.values()}
    mode=cmds.getAttr(solver.settings_group+'.rootMode')
    enabled=cmds.getAttr(solver.settings_group+'.enabled')
    input_pose=incoming_pose(solver)
    cmds.setAttr(solver.settings_group+'.enabled',True)
    def restore():
        for n,attrs in channels.items():
            for a,v in attrs.items():cmds.setAttr(n+'.'+a,*v)
        for n,attrs in weights.items():
            for a,v in attrs.items():cmds.setAttr(n+'.'+a,v)
        for n,v in stiff.items():cmds.setAttr(n+'.rotationStiffness',v)
        cmds.setAttr(solver.settings_group+'.rootMode',mode)
    def measure(name,goal=None,offset=None,rotate=0.,pa=1.,ra=1.,stiffness=None,root_mode=None,absolute=False):
        restore()
        if stiffness is not None:
            for n in stiff:cmds.setAttr(n+'.rotationStiffness',stiffness)
        if root_mode is not None:cmds.setAttr(solver.settings_group+'.rootMode',root_mode)
        if goal:
            info=solver.goal_locators[goal];loc=info['locator_dag']
            cmds.setAttr(loc+'.positionAlpha',pa);cmds.setAttr(loc+'.rotationAlpha',ra)
            cmds.setAttr(loc+'.previewMode',1 if absolute else 0)
            if offset:set_world_transform(loc,info['init_pos']+om.MVector(*offset))
            if rotate:set_world_transform(loc,quat=(info['init_rot']*om.MQuaternion(math.radians(rotate),om.MVector(0,1,0))).normal())
        # Demand a joint world matrix; do not explicitly call the solver. This
        # validates DG dirty propagation from controls and scene settings.
        pose={n:get_world_transform(n) for n in solver.skeleton.joint_names}
        result={'name':name,'residuals':{},'solve_ms':cmds.getAttr(solver.fbik_node+'.solveMilliseconds')}
        for g,info in solver.goal_locators.items():
            loc=info['locator_dag'];p,q=pose[info['bone_dag']];ip,iq=input_pose[info['bone_dag']]
            gp,gq=get_world_transform(loc)
            if cmds.getAttr(loc+'.previewMode')==0:
                gp=ip+gp-info['init_pos'];gq=(iq*info['init_rot'].inverse()*gq).normal()
            gp=ip+(gp-ip)*cmds.getAttr(loc+'.positionAlpha')
            gq=slerp(iq,gq,cmds.getAttr(loc+'.rotationAlpha'))
            result['residuals'][g]={'position_cm':(p-gp).length(),'rotation_degrees':angle(q,gq)}
        result['root_position_delta_cm']=(pose[solver.pelvis_dag][0]-input_pose[solver.pelvis_dag][0]).length()
        return result,pose
    result={'solver':solver.solver_key,'profile':solver.char_profile['template_name'],'tests':[]}
    try:
        cases=[('rest',{}),('left_hand_forward_20',{'goal':'LeftHandIK','offset':[0,0,20]}),
            ('left_hand_forward_60',{'goal':'LeftHandIK','offset':[0,0,60]}),
            ('left_foot_up_20',{'goal':'LeftFootIK','offset':[0,20,0]}),
            ('hand_rotation_60_alpha_half',{'goal':'LeftHandIK','rotate':60,'ra':.5}),
            ('foot_rotation_60_alpha_half',{'goal':'LeftFootIK','rotate':60,'ra':.5}),
            ('rotation_only_position_alpha_zero',{'goal':'LeftHandIK','rotate':60,'pa':0}),
            ('absolute_hand_forward_20',{'goal':'LeftHandIK','offset':[0,0,20],'absolute':True}),
            ('pin_root_hand_forward_20',{'goal':'LeftHandIK','offset':[0,0,20],'root_mode':1})]
        for name,kwargs in cases:
            test,_=measure(name,**kwargs);result['tests'].append(test)
        _,a=measure('stiffness_0',goal='LeftHandIK',offset=[0,0,60],stiffness=0.)
        _,b=measure('stiffness_1',goal='LeftHandIK',offset=[0,0,60],stiffness=1.)
        result['stiffness_0_to_1_max_joint_difference_cm']=max((a[n][0]-b[n][0]).length() for n in a)
        tests={t['name']:t for t in result['tests']}
        result['checks']={
            'rest_position':max(r['position_cm'] for r in tests['rest']['residuals'].values())<1e-5,
            'unmoved_right_hand':tests['left_hand_forward_20']['residuals']['RightHandIK']['position_cm']<3.,
            'rotation_alphas':all(max(r['rotation_degrees'] for r in tests[n]['residuals'].values())<1e-4
                for n in ['hand_rotation_60_alpha_half','foot_rotation_60_alpha_half','rotation_only_position_alpha_zero']),
            'stiffness_changes_pose':result['stiffness_0_to_1_max_joint_difference_cm']>1.,
            'pin_root':tests['pin_root_hand_forward_20']['root_position_delta_cm']<1e-5,
            'positive_solve_time':all(t['solve_ms']>0 for t in result['tests'])}
    finally:
        restore();cmds.setAttr(solver.settings_group+'.enabled',enabled)
        get_world_transform(solver.pelvis_dag)
    return result

def animated_fixture():
    """Animated local input, external parent motion, no-offset additive goal."""
    original_curves=set(cmds.ls(type='animCurve') or [])
    group=cmds.createNode('transform',name='__UE58_acceptance_fixture',skipSelect=True)
    created=[group]
    try:
        parent=cmds.createNode('transform',parent=group,skipSelect=True)
        joints=[]
        for i in range(3):
            j=cmds.createNode('joint',parent=joints[-1] if joints else parent,skipSelect=True)
            cmds.setAttr(j+'.tx',10. if i else 0.);joints.append(j)
        goal=cmds.createNode('transform',parent=group,skipSelect=True);cmds.setAttr(goal+'.tx',20.)
        node=cmds.createNode('ue58PBIKSolver',skipSelect=True);created.append(node)
        cmds.setAttr(node+'.enabled',False)
        identity=list(om.MMatrix())
        cfg={'parents':[-1,0,1],'root':0,'goals':[{'name':'test','bone':2,'initial_position':[20,0,0],'initial_rotation':[0,0,0,1]}],
            'native_reference':[to_engine_pose([10*i,0,0],[0,0,0,1]) for i in range(3)],
            'baseline_offsets':[identity]*3,'alignment_rotations':[None]*3}
        cmds.setAttr(node+'.configuration',json.dumps(cfg),type='string')
        for source,target in [(goal+'.worldMatrix[0]','goalMatrix[0]'),
                              (parent+'.worldMatrix[0]','externalParentMatrix'),('time1.outTime','inputTime')]:
            cmds.connectAttr(source,node+'.'+target)
        for i,j in enumerate(joints):
            cmds.connectAttr(j+'.matrix',node+'.inputLocalMatrix[{}]'.format(i))
            cmds.connectAttr(node+'.outputOffset[{}]'.format(i),j+'.offsetParentMatrix')
        cmds.setAttr(node+'.enabled',True)
        cmds.setKeyframe(joints[0],at='tx',time=1,value=2.)
        cmds.setKeyframe(joints[0],at='tx',time=2,value=12.)
        samples=[]
        for frame,translation in [(1,0),(2,0),(2,5)]:
            cmds.currentTime(frame,e=True);cmds.setAttr(parent+'.ty',translation)
            p,_=get_world_transform(joints[2]);expected=om.MVector(22. if frame==1 else 32.,translation,0)
            samples.append({'frame':frame,'parent_translation_y':translation,'position_error_cm':(p-expected).length()})
        cycle_messages=cmds.cycleCheck(joints+[node],list=True) or []
        # The hidden compatibility matrix from old scenes must have no effect,
        # even when forced dirty and supplied a large translation.
        legacy=list(identity);legacy[13]=-500.
        cmds.setAttr(node+'.pelvisMatrix',*legacy,type='matrix')
        cmds.dgdirty(node,allPlugs=True)
        p,_=get_world_transform(joints[2])
        legacy_error=(p-om.MVector(32.,5.,0.)).length()
        return {'samples':samples,'cycles':cycle_messages,'checks':{
            'animated_input_and_parent_motion':all(s['position_error_cm']<1e-5 for s in samples),
            'no_dependency_cycle':not cycle_messages,'legacy_pelvis_input_has_no_effect':legacy_error<1e-5}}
    finally:
        cmds.delete([n for n in created if cmds.objExists(n)])
        remaining=set(cmds.ls(type='animCurve') or [])-original_curves
        if remaining:cmds.delete(list(remaining))

selection=cmds.ls(sl=True,long=True) or []
frame=cmds.currentTime(q=True)
before=animation()
joints=[n for s in _ACTIVE_FBIK_SOLVERS.values() for n in s.skeleton.joint_names]
channels={n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate','scale']} for n in joints}
messages=[]
callback=om.MCommandMessage.addCommandOutputCallback(lambda m,k,d:messages.append(str(m)) if k==om.MCommandMessage.kError else None)
cmds.undoInfo(openChunk=True,chunkName='UE58 PBIK reversible acceptance')
try:
    assert cmds.about(version=True)=='2024'
    results={'maya_version':'2024','solvers':[validate_rig(s) for s in _ACTIVE_FBIK_SOLVERS.values()],
             'animation_input':animated_fixture()}
finally:
    cmds.currentTime(frame,e=True)
    cmds.select(selection,r=True) if selection else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
    om.MMessage.removeCallback(callback)
results['animation_preserved']=before==animation()
results['joint_channels_preserved']=channels=={n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate','scale']} for n in joints}
results['maya_errors']=messages
results['passed']=all(v for s in results['solvers'] for v in s['checks'].values()) and all(results['animation_input']['checks'].values()) and results['animation_preserved'] and results['joint_channels_preserved'] and not messages
HERE.joinpath('maya_pbik_results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results))
assert results['passed'],'Live Maya acceptance failed; see maya_pbik_results.json'
