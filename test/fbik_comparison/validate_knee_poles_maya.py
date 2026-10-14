"""Live knee-direction, straight-limb and axial-roll checks; no scene save."""
import json
import math
from pathlib import Path
import maya.cmds as cmds
import maya.api.OpenMaya as om
from ue_ik_retargeter_maya.core.fbik_scene import _ACTIVE_FBIK_SOLVERS
from ue_ik_retargeter_maya.core.math_utils import get_world_transform,set_world_transform
from ue_ik_retargeter_maya.core.pole_vector import build_reference,resolve_reference,rotate,bend_measure
from ue_ik_retargeter_maya.core.ik_solver import TwoBoneIKSolver,AlignPoleVectorSolver,IKChainSolver
from ue_ik_retargeter_maya.core.chains import AutoCharacterizer

HERE=Path(__file__).resolve().parent
selection=cmds.ls(sl=True,long=True) or [];frame=cmds.currentTime(q=True)
curves={n:{'times':cmds.keyframe(n,q=True,timeChange=True),'values':cmds.keyframe(n,q=True,valueChange=True),
           'destinations':cmds.listConnections(n+'.output',s=False,d=True,p=True)} for n in cmds.ls(type='animCurve')}
joints=[n for s in _ACTIVE_FBIK_SOLVERS.values() for n in s.skeleton.joint_names]
channels={n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate','scale']} for n in joints}
results={'rigs':{},'analytic_checks':{}}
errors=[]
callback=om.MCommandMessage.addCommandOutputCallback(lambda m,k,d:errors.append(str(m)) if k==om.MCommandMessage.kError else None)
cmds.undoInfo(openChunk=True,chunkName='UE58 stable knee pole acceptance')
try:
    import ue_ik_retargeter_maya as plugin
    before_nodes={k:s.fbik_node for k,s in _ACTIVE_FBIK_SOLVERS.items()}
    for repeat in range(2):
        plugin.reload_plugin()
        from ue_ik_retargeter_maya.core.pbik_native import NativePBIKSolver
        for key,s in _ACTIVE_FBIK_SOLVERS.items():
            s._solve_internal()
            selection_list=om.MSelectionList();selection_list.add(s.fbik_node)
            instance=om.MFnDependencyNode(selection_list.getDependNode(0)).userNode()
            assert type(instance.native) is NativePBIKSolver
    assert before_nodes=={k:s.fbik_node for k,s in _ACTIVE_FBIK_SOLVERS.items()}
    results['hot_reload_checks']={'node_registry_preserved':True,'native_handles_recreated':True}
    from ue_ik_retargeter_maya.core.ik_solver import TwoBoneIKSolver,AlignPoleVectorSolver,IKChainSolver
    from ue_ik_retargeter_maya.core.chains import AutoCharacterizer
    # Regenerating asset profiles must retain the calibrated anatomical sign;
    # this is separate from changing the already assembled scene parameters.
    for s in _ACTIVE_FBIK_SOLVERS.values():
        profile=AutoCharacterizer.characterize(s.skeleton)
        for name,settings in profile['fbik_bone_settings'].items():
            if 'pole_calibration' in settings:
                assert settings['preferred_angles'][0]<0.,(name,settings)
    results['analytic_checks']['generated_profiles_keep_correct_sign']=True
    for key,s in _ACTIVE_FBIK_SOLVERS.items():
        # Change the actual input skeleton, with world-space feet/hands pinned.
        # Temporarily isolate translation drivers to avoid editing their keys.
        root=s.pelvis_dag;original_translate=cmds.getAttr(root+'.translate')[0]
        raw=cmds.listConnections(root+'.translate',s=True,d=False,p=True,c=True) or []
        connections=[(raw[i+1],raw[i]) for i in range(0,len(raw),2)]
        modes={g['locator_dag']:cmds.getAttr(g['locator_dag']+'.previewMode') for g in s.goal_locators.values()}
        cfg=json.loads(cmds.getAttr(s.fbik_node+'.configuration'))
        root_index=s.skeleton.joint_indices[root];parent_index=cfg['parents'][root_index]
        parent_world=om.MMatrix(cmds.getAttr(s.skeleton.joint_names[parent_index]+'.worldMatrix[0]')) if parent_index>=0 else om.MMatrix(cmds.getAttr(s.fbik_node+'.externalParentMatrix'))
        translation_space=om.MMatrix(cfg['baseline_offsets'][root_index])*parent_world
        auto_key=cmds.autoKeyframe(q=True,state=True)
        references={}
        for side in ['Left','Right']:
            chain=s.bone_chains[side+'Leg'];knee=s.skeleton.get_clean_name(build_reference(chain)['bones'][1])
            references[side]=build_reference(chain,settings=s.fbik_settings[knee])
        report=[]
        try:
            cmds.autoKeyframe(state=False)
            for source,destination in connections:cmds.disconnectAttr(source,destination)
            for control in modes:cmds.setAttr(control+'.previewMode',1)
            for down in [10.,20.]:
                delta=om.MVector(0,-down,0)*translation_space.inverse()
                cmds.setAttr(root+'.translate',*(om.MVector(*original_translate)+delta))
                sample={'input_pelvis_down_cm':down,'knees':{}}
                for side,ref in references.items():
                    a,b,c=ref['bones'];pa,qa=get_world_transform(a);pb,_=get_world_transform(b);pc,_=get_world_transform(c)
                    front=om.MVector(*rotate(ref['local_direction'],list(qa)))
                    measure=bend_measure(list(pa),list(pb),list(pc))
                    alignment=om.MVector(*measure['pole'])*front
                    sample['knees'][side]={'forward_distance_cm':alignment,'bend_angle_degrees':measure['angle_degrees'],'reference_source':ref['source']}
                    assert alignment>1.,(key,side,sample)
                report.append(sample)
        finally:
            cmds.setAttr(root+'.translate',*original_translate)
            for source,destination in connections:cmds.connectAttr(source,destination)
            for control,mode in modes.items():cmds.setAttr(control+'.previewMode',mode)
            cmds.autoKeyframe(state=auto_key)
        results['rigs'][key]=report

    # A fully straight limb, with tiny offsets on either side of its chord,
    # must select the configured forward pole in the analytical IK path.
    for noise in [-1e-8,0.,1e-8]:
        a,b,c=om.MVector(0,0,0),om.MVector(0,-1,noise),om.MVector(0,-2,0)
        out=TwoBoneIKSolver.solve(a,b,c,om.MQuaternion(),om.MQuaternion(),om.MQuaternion(),
            om.MVector(0,-1.5,0),om.MQuaternion(),None,1.,1.,reference_pole_direction=om.MVector(0,0,1))
        assert out[3].z>.5,out[3]
    results['analytic_checks']['straight_and_opposite_noise_forward']=True
    ik=IKChainSolver.__new__(IKChainSolver)
    ik.src_pole_reference={'bones':['a','b','c'],'local_chord':[0,-1,0],'local_direction':[1,0,0],'orientation_known':False}
    pose={n:(p,om.MQuaternion()) for n,p in zip(['a','b','c'],[om.MVector(),om.MVector(0,-1,.2),om.MVector(0,-2,0)])}
    assert ik.pole_direction(pose).z>.999
    results['analytic_checks']['uncalibrated_limb_uses_clear_animated_bend']=True

    # An opposite pole is a signed 180deg rotation about the limb axis. It must
    # not move the endpoint as a generic antiparallel-vector rotation can do.
    a,b,c=om.MVector(0,0,0),om.MVector(0,-1,-.2),om.MVector(0,-2,0)
    r0,r1,knee,q=AlignPoleVectorSolver.align_limb_pole_vector(
        a,om.MVector(0,-1,0),c,a,b,c,om.MQuaternion(),om.MQuaternion(),
        src_reference_direction=om.MVector(0,0,1),tgt_reference_direction=om.MVector(0,0,1))
    assert knee.z>.19
    assert (c.rotateBy(q)-c).length()<1e-8
    results['analytic_checks']['opposite_pole_axial_roll_preserves_endpoint']=True
finally:
    cmds.currentTime(frame,e=True)
    cmds.select(selection,r=True) if selection else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
    om.MMessage.removeCallback(callback)
after={n:{'times':cmds.keyframe(n,q=True,timeChange=True),'values':cmds.keyframe(n,q=True,valueChange=True),
          'destinations':cmds.listConnections(n+'.output',s=False,d=True,p=True)} for n in cmds.ls(type='animCurve')}
results['animation_preserved']=curves==after
results['joint_channels_preserved']=channels=={n:{a:cmds.getAttr(n+'.'+a)[0] for a in ['translate','rotate','scale']} for n in joints}
assert results['animation_preserved'] and results['joint_channels_preserved']
results['maya_errors']=errors
assert not errors,errors
results['passed']=True
HERE.joinpath('knee_pole_results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results))
