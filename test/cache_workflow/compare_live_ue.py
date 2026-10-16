"""Compare retained cached rigs with UE output for the identical FBX clip."""
import json
import math
import __main__
from pathlib import Path
import maya.cmds as cmds
import maya.api.OpenMaya as om
from ue_ik_retargeter_maya.core.cache_workflow import _cached_joints,_rig_order,scene_state
from ue_ik_retargeter_maya.core.ue_reference import apply_ue_profile
from ue_ik_retargeter_maya.core.pbik_native import from_engine_pose
from ue_ik_retargeter_maya.core.math_utils import matrix_to_trs

HERE=Path(__file__).parent
profile=json.loads((HERE/'ue_profile.json').read_text(encoding='utf-8'))
golden=json.loads((HERE/'ue_reference.json').read_text(encoding='utf-8'))
session=__main__._ue58_cache_session
manifest=__main__._ue58_cache_build['manifest']
config=json.loads(cmds.getAttr(session['target_node']+'.configuration'))
config['retarget_bundle']=apply_ue_profile(manifest['bundle'],profile)
with scene_state(undo_name='Apply UE retarget profile to cached rigs'):
    driver=cmds.listConnections(session['source_node']+'.enabled',s=True,d=False,p=True)[0]
    cmds.setAttr(driver,False)
    for role,rig,suffix in [('Source',config['retarget_bundle']['src_ikrig'],0),
        ('Target',config['retarget_bundle']['tgt_ikrig'],config['source_count'])]:
        ordered=_rig_order(rig,_cached_joints(session['systems'][role]))
        for i,joint in enumerate(ordered):
            cmds.connectAttr(joint+('.worldMatrix[0]' if role=='Source' else '.matrix'),
                session['target_node']+'.inputLocalMatrix[{}]'.format(i+suffix),force=True)
    cmds.setAttr(session['target_node']+'.configuration',json.dumps(config),type='string')
report={'source_clip':golden['animation'],'reference_rtg':profile['asset'],'roles':{}}
for role in ['Source','Target']:
    joints=_cached_joints(session['systems'][role])
    names={j.split('|')[-1].split(':')[-1]:j for j in joints}
    results=[]
    with scene_state(temporary=True):
        for frame,engine in golden['frames'][role].items():
            cmds.currentTime(int(frame),e=True)
            display=om.MMatrix(cmds.xform(session['systems'][role],q=True,ws=True,matrix=True)).inverse()
            errors=[]
            for name,joint in names.items():
                if name not in engine:continue
                p,q,_=matrix_to_trs(om.MMatrix(cmds.xform(joint,q=True,ws=True,matrix=True))*display)
                value=engine[name]
                ep,eq,_=from_engine_pose(value['translation']+value['rotation']+value['scale3d'])
                eq=om.MQuaternion(*eq).normal()
                angle=math.degrees(2*math.acos(min(1.,abs(sum(a*b for a,b in zip(q,eq))))))
                errors.append({'bone':name,'position_cm':(p-om.MVector(*ep)).length(),'rotation_degrees':angle})
            results.append({'frame':int(frame),'position':max(errors,key=lambda x:x['position_cm']),
                'rotation':max(errors,key=lambda x:x['rotation_degrees'])})
    report['roles'][role]={'bones':len(errors),'samples':results,
        'max_position_cm':max(x['position']['position_cm'] for x in results),
        'max_rotation_degrees':max(x['rotation']['rotation_degrees'] for x in results)}
__main__._ue58_engine_profile=profile
__main__._ue58_profile_bundle=config['retarget_bundle']
__main__._ue58_golden_comparison=report
(HERE/'maya_ue_comparison.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:{n:v for n,v in r.items() if n!='samples'} for k,r in report['roles'].items()}))
