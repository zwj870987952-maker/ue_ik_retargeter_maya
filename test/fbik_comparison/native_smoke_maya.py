"""Small standalone Maya test for PBIK node wiring (no user scene access)."""
import sys,json
from pathlib import Path
import maya.standalone
maya.standalone.initialize(name='python')
import maya.cmds as cmds
import maya.api.OpenMaya as om
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ue58_three_assets_v2'))
from ue_ik_retargeter_maya.core.pbik_native import to_engine_pose,library
library()
print('NATIVE_LOADED',flush=True)
cmds.loadPlugin(str(ROOT/'ue58_three_assets_v2/ue_ik_retargeter_maya/core/ue58_pbik_node.py'),quiet=True)
print('NODE_REGISTERED',flush=True)
joints=[]
for i in range(3):
    cmds.select(joints[-1] if joints else [],r=True)
    joint=cmds.joint(name='probe_joint_'+str(i),position=(i*10,0,0));joints.append(joint)
goal=cmds.spaceLocator(name='probe_goal')[0];cmds.setAttr(goal+'.tx',20)
node=cmds.createNode('ue58PBIKSolver');cmds.setAttr(node+'.enabled',False)
identity=list(om.MMatrix())
cfg={'parents':[-1,0,1],'root':0,'goals':[{'name':'test','bone':2,'initial_position':[20,0,0],'initial_rotation':[0,0,0,1]}],
    'native_reference':[to_engine_pose([i*10,0,0],[0,0,0,1]) for i in range(3)],
    'baseline_offsets':[identity]*3,'alignment_rotations':[None]*3}
cmds.setAttr(node+'.configuration',json.dumps(cfg),type='string')
cmds.connectAttr(goal+'.worldMatrix[0]',node+'.goalMatrix[0]')
for i,j in enumerate(joints):cmds.connectAttr(j+'.matrix',node+'.inputLocalMatrix[{}]'.format(i))
print('INPUTS_CONNECTED',flush=True)
for i,j in enumerate(joints):
    cmds.connectAttr(node+'.outputOffset[{}]'.format(i),j+'.offsetParentMatrix')
    print('OUTPUT_CONNECTED',i,flush=True)
cmds.setAttr(node+'.enabled',True)
print('ENABLED',flush=True)
print('REST',cmds.xform(joints[-1],q=True,ws=True,t=True),flush=True)
cmds.setAttr(goal+'.ty',5)
print('MOVED',cmds.xform(joints[-1],q=True,ws=True,t=True),flush=True)
print('CYCLES',cmds.cycleCheck(joints,list=True),flush=True)
print('SMOKE_DONE',flush=True)
maya.standalone.uninitialize()
