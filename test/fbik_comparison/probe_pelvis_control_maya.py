"""Measure the custom pelvis input without editing user controls or keys."""
import json
from pathlib import Path
import maya.cmds as cmds
import maya.api.OpenMaya as om
from ue_ik_retargeter_maya.core.fbik_scene import _ACTIVE_FBIK_SOLVERS
from ue_ik_retargeter_maya.core.math_utils import get_world_transform

HERE=Path(__file__).resolve().parent
selection=cmds.ls(sl=True,long=True) or [];frame=cmds.currentTime(q=True)
curves={n:(cmds.keyframe(n,q=True,timeChange=True),cmds.keyframe(n,q=True,valueChange=True)) for n in cmds.ls(type='animCurve')}
result={'solvers':{}}
cmds.undoInfo(openChunk=True,chunkName='UE58 custom pelvis input inspection')
try:
    for key,s in _ACTIVE_FBIK_SOLVERS.items():
        node=s.fbik_node;plug=node+'.pelvisMatrix';group=s.settings_group
        original_matrix=cmds.getAttr(plug)
        sources=cmds.listConnections(plug,s=True,d=False,p=True) or []
        assert len(sources)==1,(plug,sources)
        root_mode=cmds.getAttr(group+'.rootMode')
        cfg=json.loads(cmds.getAttr(node+'.configuration'))
        def pose():
            cmds.getAttr(node+'.outputOffset[0]')
            return {n:get_world_transform(n)[0] for n in s.skeleton.joint_names}
        original_pose=pose()
        initial=list(cfg['pelvis_initial_matrix']);shifted=list(initial);shifted[13]-=10.
        initial_p=om.MVector(*initial[12:15]);current_p=om.MVector(*original_matrix[12:15])
        try:
            cmds.disconnectAttr(sources[0],plug)
            cmds.setAttr(plug,*initial,type='matrix');neutral=pose()
            cmds.setAttr(plug,*shifted,type='matrix');translated=pose()
            cmds.setAttr(group+'.rootMode',1)
            cmds.setAttr(plug,*initial,type='matrix');pin_neutral=pose()
            cmds.setAttr(plug,*shifted,type='matrix');pin_translated=pose()
            result['solvers'][key]={
                'native_goal_count':len(cfg['goals']),
                'current_control_offset_cm':(current_p-initial_p).length(),
                'neutral_vs_current_max_joint_difference_cm':max((neutral[n]-original_pose[n]).length() for n in neutral),
                'pelvis_control_down_10_cm':{
                    'root_displacement_cm':(translated[s.pelvis_dag]-neutral[s.pelvis_dag]).length(),
                    'max_joint_displacement_cm':max((translated[n]-neutral[n]).length() for n in neutral)},
                'pin_to_input_with_control_down_10_cm':{
                    'root_displacement_cm':(pin_translated[s.pelvis_dag]-pin_neutral[s.pelvis_dag]).length()},
                'goal_locators_edited':False}
        finally:
            cmds.setAttr(group+'.rootMode',root_mode)
            cmds.setAttr(plug,*original_matrix,type='matrix')
            cmds.connectAttr(sources[0],plug)
        restored=pose()
        error=max((restored[n]-original_pose[n]).length() for n in restored)
        result['solvers'][key]['restore_max_joint_difference_cm']=error
        assert error<1e-7,error
finally:
    cmds.currentTime(frame,e=True)
    cmds.select(selection,r=True) if selection else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
result['animation_preserved']=curves=={n:(cmds.keyframe(n,q=True,timeChange=True),cmds.keyframe(n,q=True,valueChange=True)) for n in cmds.ls(type='animCurve')}
assert result['animation_preserved']
HERE.joinpath('maya_pelvis_control_effects.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
