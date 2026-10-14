"""Build an isolated rig with the production builder; leave user rigs intact."""
import json
from pathlib import Path
import maya.cmds as cmds
from ue_ik_retargeter_maya.core.skeleton import RetargetSkeleton
from ue_ik_retargeter_maya.core.chains import AutoCharacterizer
from ue_ik_retargeter_maya.core.retarget_assets import IKRigAsset,MayaSceneAssetBuilder
from ue_ik_retargeter_maya.core.fbik_scene import _ACTIVE_FBIK_SOLVERS,clear_fbik_callbacks

namespace='__UE58FourGoalTest';key='FourGoalBuilderTest'
assert not cmds.namespace(exists=namespace)
selection=cmds.ls(sl=True,long=True) or [];frame=cmds.currentTime(q=True)
before_keys={n:(cmds.keyframe(n,q=True,timeChange=True),cmds.keyframe(n,q=True,valueChange=True)) for n in cmds.ls(type='animCurve')}
before_registry=set(_ACTIVE_FBIK_SOLVERS)
group=None
cmds.undoInfo(openChunk=True,chunkName='UE58 four goal builder acceptance')
try:
    cmds.namespace(add=namespace)
    group=cmds.createNode('transform',name='__UE58FourGoalTestGroup',skipSelect=True)
    root=cmds.createNode('joint',name=namespace+':Hips',parent=group,skipSelect=True)
    cmds.setAttr(root+'.ty',100.)
    for side,sign in [('Left',1.),('Right',-1.)]:
        parent=root
        for name,translation in [(side+'UpLeg',(8.*sign,-10.,0.)),(side+'Leg',(0.,-40.,.2)),
                                 (side+'Foot',(0.,-40.,-.2)),(side+'ToeBase',(0.,-5.,12.))]:
            joint=cmds.createNode('joint',name=namespace+':'+name,parent=parent,skipSelect=True)
            cmds.setAttr(joint+'.translate',*translation);parent=joint
        parent=root
        for name,translation in [(side+'Arm',(10.*sign,40.,0.)),(side+'ForeArm',(20.*sign,0.,0.)),(side+'Hand',(20.*sign,0.,0.))]:
            joint=cmds.createNode('joint',name=namespace+':'+name,parent=parent,skipSelect=True)
            cmds.setAttr(joint+'.translate',*translation);parent=joint
    skeleton=RetargetSkeleton(root);profile=AutoCharacterizer.characterize(skeleton)
    asset=IKRigAsset('IK_FourGoalBuilderTest','Target',skeleton,profile)
    asset_node,solver=MayaSceneAssetBuilder._build_ikrig_scene_node(group,asset,key,None,'',(.9,.8,.1),(.9,.8,.1))
    cfg=json.loads(cmds.getAttr(solver.fbik_node+'.configuration'))
    assert len(solver.goal_locators)==4
    assert len(cmds.listRelatives(asset_node,ad=True,type='locator',fullPath=True) or [])==4
    assert cfg['root']==skeleton.joint_indices[solver.pelvis_dag]
    assert 'pelvis_initial_matrix' not in cfg and 'solving_branch' not in cfg
    assert not cmds.listConnections(solver.fbik_node+'.pelvisMatrix',s=True,d=False)
    report={'goal_count':4,'locator_count':4,'solver_root_preserved':True,
            'legacy_pelvis_connection':False,'profile':profile['template_name']}
finally:
    clear_fbik_callbacks(key)
    if group and cmds.objExists(group):cmds.delete(group)
    if cmds.namespace(exists=namespace):cmds.namespace(removeNamespace=namespace)
    cmds.currentTime(frame,e=True)
    cmds.select(selection,r=True) if selection else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
assert before_registry==set(_ACTIVE_FBIK_SOLVERS)
assert before_keys=={n:(cmds.keyframe(n,q=True,timeChange=True),cmds.keyframe(n,q=True,valueChange=True)) for n in cmds.ls(type='animCurve')}
report['animation_preserved']=True;report['passed']=True
Path(__file__).with_name('four_goal_builder_results.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
