"""Reversible measurements of the existing live FBIK implementation.

Goals, joint channels, animation curves, current time and selection are restored
in finally blocks. Reports goal residuals rather than claiming UE pose parity.
"""
import json
import math
import time
from pathlib import Path
import maya.cmds as cmds
import maya.api.OpenMaya as om
from ue_ik_retargeter_maya.core.fbik_solver import _ACTIVE_FBIK_SOLVERS
from ue_ik_retargeter_maya.core.math_utils import get_world_transform, set_world_transform, slerp

HERE = Path(__file__).resolve().parent

def xyz(vector):
    return [vector.x,vector.y,vector.z]

def angle(a,b):
    delta = (a.inverse()*b).normal()
    return math.degrees(2*math.acos(max(-1.,min(1.,abs(delta.w)))))

def curves():
    return {n:{'times':cmds.keyframe(n,q=True,timeChange=True),
               'values':cmds.keyframe(n,q=True,valueChange=True)}
            for n in cmds.ls(type='animCurve') or []}

def probe(solver):
    nodes = list(solver.skeleton.joint_names)+[i['locator_dag'] for i in solver.goal_locators.values()]
    if solver.pelvis_locator_dag:
        nodes.append(solver.pelvis_locator_dag)
    saved = {n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate','scale']} for n in nodes}
    weights = {i['locator_dag']:{a:cmds.getAttr(i['locator_dag']+'.'+a) for a in ['positionAlpha','rotationAlpha']} for i in solver.goal_locators.values()}
    settings = {n:dict(v) for n,v in solver.fbik_settings.items()}
    solver._is_solving = True
    animation_connections=[]
    for n in solver.skeleton.joint_names:
        for a in ['tx','ty','tz','rx','ry','rz','sx','sy','sz']:
            destination=n+'.'+a
            for source in cmds.listConnections(destination,s=True,d=False,plugs=True,type='animCurve') or []:
                animation_connections.append((source,destination))
                cmds.disconnectAttr(source,destination)
    def restore():
        # Restore topological parent-first transforms, including driven channels.
        for n in nodes:
            for a,v in saved[n].items():
                cmds.setAttr(n+'.'+a,*v)
        for n,attrs in weights.items():
            for a,v in attrs.items():cmds.setAttr(n+'.'+a,v)
        solver.fbik_settings.clear()
        solver.fbik_settings.update({n:dict(v) for n,v in settings.items()})
    def measure(name, goal_name=None, offset=None, rotation_degrees=0, rotation_alpha=1., stiffness=None, position_alpha=1.):
        restore()
        if stiffness is not None:
            for v in solver.fbik_settings.values():v['rotation_stiffness']=stiffness
        if goal_name:
            info=solver.goal_locators[goal_name]
            cmds.setAttr(info['locator_dag']+'.positionAlpha',position_alpha)
            if offset:
                set_world_transform(info['locator_dag'], info['init_pos']+om.MVector(*offset))
            if rotation_degrees:
                q=(info['init_rot']*om.MQuaternion(math.radians(rotation_degrees),om.MVector(0,1,0))).normal()
                set_world_transform(info['locator_dag'],quat=q)
                cmds.setAttr(info['locator_dag']+'.rotationAlpha',rotation_alpha)
        start=time.perf_counter()
        solver._solve_internal()
        elapsed=(time.perf_counter()-start)*1000
        result={'name':name,'goal':goal_name,'offset_cm':offset,'rotation_degrees':rotation_degrees,
                'rotation_alpha':rotation_alpha,'solve_ms':elapsed,'residuals':{}}
        pelvis_pos,_=get_world_transform(solver.pelvis_dag)
        result['pelvis_delta_cm']=xyz(pelvis_pos-solver.base_world_pos[solver.pelvis_dag])
        result['world_positions']={}
        for n in solver.skeleton.joint_names:
            p,_=get_world_transform(n);result['world_positions'][n]=xyz(p)
        for name,info in solver.goal_locators.items():
            pos,rot=get_world_transform(info['bone_dag'])
            gp,gr=get_world_transform(info['locator_dag'])
            pa=cmds.getAttr(info['locator_dag']+'.positionAlpha');ra=cmds.getAttr(info['locator_dag']+'.rotationAlpha')
            expected_pos=info['init_pos']+(gp-info['init_pos'])*pa
            expected_rot=slerp(info['init_rot'],gr,ra)
            result['residuals'][name]={'position_cm':(pos-expected_pos).length(),
                'rotation_degrees':angle(rot,expected_rot),'bone_position':xyz(pos),'goal_position':xyz(expected_pos)}
        return result
    result={'solver':solver.solver_key,'profile':solver.char_profile['template_name'],'tests':[]}
    try:
        result['tests'].append(measure('rest'))
        result['tests'].append(measure('left_hand_forward_20','LeftHandIK',[0,0,20]))
        result['tests'].append(measure('left_hand_forward_60','LeftHandIK',[0,0,60]))
        result['tests'].append(measure('left_foot_up_20','LeftFootIK',[0,20,0]))
        result['tests'].append(measure('hand_rotation_60_alpha_half','LeftHandIK',rotation_degrees=60,rotation_alpha=.5))
        result['tests'].append(measure('foot_rotation_60_alpha_half','LeftFootIK',rotation_degrees=60,rotation_alpha=.5))
        result['tests'].append(measure('rotation_only_position_alpha_zero','LeftHandIK',rotation_degrees=60,position_alpha=0.))
        a=measure('all_stiffness_0','LeftHandIK',[0,0,60],stiffness=0.)
        b=measure('all_stiffness_1','LeftHandIK',[0,0,60],stiffness=1.)
        result['stiffness_0_to_1_max_joint_difference_cm']=max(
            (om.MVector(*a['world_positions'][n])-om.MVector(*b['world_positions'][n])).length() for n in a['world_positions'])
        for test in result['tests']:test.pop('world_positions')
    finally:
        restore()
        for source,destination in animation_connections:
            cmds.connectAttr(source,destination)
        solver._is_solving=False
    result['restore_max_channel_difference']=max(abs(cmds.getAttr(n+'.'+a)[0][i]-v[i])
        for n,attrs in saved.items() for a,v in attrs.items() for i in range(3))
    return result

def probe_animation_input(frame=80):
    solvers=list(_ACTIVE_FBIK_SOLVERS.values())
    target=next(s for s in solvers if s.solver_key.startswith('Tgt_'))
    original_time=cmds.currentTime(q=True)
    nodes=[n for s in solvers for n in s.skeleton.joint_names]
    saved={n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate']} for n in nodes}
    for s in solvers:s._is_solving=True
    try:
        cmds.currentTime(frame,e=True)
        input_pose={n:get_world_transform(n) for n in target.skeleton.joint_names}
        target._solve_internal()
        differences={n:(get_world_transform(n)[0]-p[0]).length() for n,p in input_pose.items()}
        return {'frame':frame,'all_goals_unmoved':True,
                'max_input_to_output_position_change_cm':max(differences.values()),
                'pelvis_input':xyz(input_pose[target.pelvis_dag][0]),
                'pelvis_output':xyz(get_world_transform(target.pelvis_dag)[0]),
                'largest_changed_bone':max(differences,key=differences.get)}
    finally:
        cmds.currentTime(original_time,e=True)
        for n,attrs in saved.items():
            for a,v in attrs.items():cmds.setAttr(n+'.'+a,*v)
        for s in solvers:s._is_solving=False

selection=cmds.ls(sl=True,long=True) or []
current_time=cmds.currentTime(q=True)
before=curves()
cmds.undoInfo(openChunk=True,chunkName='UE58 reversible FBIK probes')
try:
    results={'maya_version':cmds.about(version=True),'scene':cmds.file(q=True,sn=True),'time':current_time,
             'solvers':[probe(s) for s in _ACTIVE_FBIK_SOLVERS.values()]}
    # Existing curves in this saved scene are static reference-pose keys.
    # Dynamic input validation needs a separate animated fixture / UE golden pose.
    results['animation_curve_count_before']=len(before)
    results['animation_curve_count_after']=len(curves())
    results['animation_curves_preserved']=before==curves()
    HERE.joinpath('maya_probe_results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(json.dumps({'solvers':[{k:v for k,v in s.items() if k!='tests'} for s in results['solvers']],
                      'animation_curves_preserved':results['animation_curves_preserved']}))
finally:
    cmds.currentTime(current_time,e=True)
    cmds.select(selection,r=True) if selection else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
