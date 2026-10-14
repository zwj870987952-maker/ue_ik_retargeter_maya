"""Migrate the open project rig to four goals, preserving channels and keys."""
import json
import math
from pathlib import Path
import maya.cmds as cmds
import ue_ik_retargeter_maya as plugin
from ue_ik_retargeter_maya.core.fbik_scene import _ACTIVE_FBIK_SOLVERS
from ue_ik_retargeter_maya.core.math_utils import get_world_transform

assert cmds.about(version=True)=='2024'
assert Path(cmds.file(q=True,sn=True)).name=='test_maya.ma'
selection=cmds.ls(sl=True,long=True) or [];frame=cmds.currentTime(q=True)
def curves():
    return {n:{'times':cmds.keyframe(n,q=True,timeChange=True),'values':cmds.keyframe(n,q=True,valueChange=True),
               'destinations':sorted(cmds.listConnections(n+'.output',s=False,d=True,p=True) or [])}
            for n in cmds.ls(type='animCurve') or []}
before_curves=curves()
before={}
for key,s in _ACTIVE_FBIK_SOLVERS.items():
    cfg=json.loads(cmds.getAttr(s.fbik_node+'.configuration'))
    control=getattr(s,'pelvis_locator_dag',None)
    assert control and cmds.objExists(control)
    before[key]={'control':control,'node':s.fbik_node,'root_index':cfg['root'],
                 'root_behavior':cmds.getAttr(s.settings_group+'.rootMode'),
                 'pose':{n:get_world_transform(n) for n in s.skeleton.joint_names},
                 'channels':{n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate','scale']} for n in s.skeleton.joint_names}}
report={'deleted_controls':[],'solvers':{},'scene_saved':False}
cmds.undoInfo(openChunk=True,chunkName='Remove UE58 extra PelvisRoot controls')
try:
    plugin.reload_plugin()
    for key,s in _ACTIVE_FBIK_SOLVERS.items():
        old=before[key];node=s.fbik_node
        cfg=json.loads(cmds.getAttr(node+'.configuration'))
        assert not cmds.objExists(old['control'])
        assert node==old['node'] and cfg['root']==old['root_index']
        assert cmds.getAttr(s.settings_group+'.rootMode')==old['root_behavior']
        assert len(cfg['goals'])==4 and len(s.goal_locators)==4
        assert 'pelvis_initial_matrix' not in cfg and 'solving_branch' not in cfg
        assert not cmds.listConnections(node+'.pelvisMatrix',s=True,d=False)
        s._solve_internal()
        after={n:get_world_transform(n) for n in s.skeleton.joint_names}
        position_error=max((after[n][0]-old['pose'][n][0]).length() for n in after)
        rotation_error=max(math.degrees(2*math.acos(min(1.,abs((after[n][1].inverse()*old['pose'][n][1]).normal().w)))) for n in after)
        assert position_error<1e-6,(key,position_error)
        assert rotation_error<1e-4,(key,rotation_error)
        assert old['channels']=={n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate','scale']} for n in s.skeleton.joint_names}
        report['deleted_controls'].append(old['control'])
        report['solvers'][key]={'node':node,'root_bone':s.pelvis_dag,'goal_count':len(s.goal_locators),
                                'max_position_change_cm':position_error,'max_rotation_change_degrees':rotation_error}
finally:
    cmds.currentTime(frame,e=True)
    kept=[n for n in selection if cmds.objExists(n)]
    cmds.select(kept,r=True) if kept else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
report['animation_preserved']=before_curves==curves()
assert report['animation_preserved']
report['passed']=True
Path(__file__).with_name('pelvis_root_removal.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
