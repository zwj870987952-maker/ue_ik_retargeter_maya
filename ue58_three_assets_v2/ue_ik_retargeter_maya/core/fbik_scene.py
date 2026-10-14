# -*- coding: utf-8 -*-
"""Scene wiring for live UE 5.8 PBIK through a Maya DG matrix node."""
import json
import importlib
from pathlib import Path
import maya.cmds as cmds
import maya.api.OpenMaya as om
from .math_utils import get_world_transform,set_world_transform,rotate_vector_by_quat
from .pbik_native import library,to_engine_pose
from .pole_vector import calibrate_knee_settings

_ACTIVE_FBIK_SOLVERS=globals().get('_ACTIVE_FBIK_SOLVERS',{})
_ACTIVE_CALLBACK_IDS=globals().get('_ACTIVE_CALLBACK_IDS',{})
DEFAULT_SOLVER_SETTINGS={'root_behavior':'Free','iterations':20,'sub_iterations':10,
    'mass_multiplier':1.,'allow_stretch':False,'global_pull_chain_alpha':0.,'max_angle':30.,'over_relaxation':1.3}

def reload_node_compute():
    """Refresh Python compute methods without deleting existing scene nodes.

    Maya owns a separate class created when loading the plugin file. Reloading
    the package alone does not update it. Attribute-schema changes still require
    a Maya restart; existing node attribute objects remain on the owned class.
    """
    nodes=cmds.ls(type='ue58PBIKSolver') if 'ue58PBIKSolver' in cmds.allNodeTypes() else []
    if not nodes:return
    module=importlib.import_module('.ue58_pbik_node',__package__)
    module=importlib.reload(module)
    for node in nodes:
        selection=om.MSelectionList();selection.add(node)
        instance=om.MFnDependencyNode(selection.getDependNode(0)).userNode()
        # Reloading pbik_native creates new ctypes.Structure classes. An old
        # handle's DLL argument types and buffers cannot accept those classes.
        # Recreate the native cache together with the Python compute method.
        if instance.native:instance.native.close()
        instance.native=None;instance.signature=None
        type(instance).compute=module.UE58PBIKNode.compute
        remove_legacy_pelvis_control(node)
        cmds.dgdirty(node,allPlugs=True)

def remove_legacy_pelvis_control(node):
    """Migrate owned controls from previous versions; keep the solver root."""
    removed=[]
    if cmds.attributeQuery('pelvisMatrix',node=node,exists=True):
        plug=node+'.pelvisMatrix'
        for source in cmds.listConnections(plug,s=True,d=False,plugs=True) or []:
            control=source.split('.')[0]
            cmds.disconnectAttr(source,plug)
            if (cmds.attributeQuery('goalName',node=control,exists=True)
                    and cmds.getAttr(control+'.goalName')=='PelvisRoot'
                    and cmds.attributeQuery('boneDAG',node=control,exists=True)):
                removed.append(control);cmds.delete(control)
        selection=om.MSelectionList();selection.add(node)
        attribute=om.MFnDependencyNode(selection.getDependNode(0)).attribute('pelvisMatrix')
        om.MFnAttribute(attribute).hidden=True
    text=cmds.getAttr(node+'.configuration')
    if text:
        config=json.loads(text)
        if 'pelvis_initial_matrix' in config or 'solving_branch' in config:
            config.pop('pelvis_initial_matrix',None);config.pop('solving_branch',None)
            cmds.setAttr(node+'.configuration',json.dumps(config),type='string')
    # Old UI instances may still check this attribute until reopened. Clear
    # their references rather than leaving a path to a deleted locator.
    for solver in _ACTIVE_FBIK_SOLVERS.values():
        if solver.fbik_node==node:
            for name in ['pelvis_locator_dag','pelvis_init_pos','pelvis_init_rot']:
                solver.__dict__.pop(name,None)
            if not isinstance(getattr(type(solver),'pelvis_locator_dag',None),property):
                solver.__dict__['pelvis_locator_dag']=None
    return removed

def effective_bone_settings(settings):
    result=dict(settings)
    preferred=result.get('preferred_angles')
    if preferred is None:
        preferred=[0.,0.,0.];axis=result.get('preferred_axis','')
        if axis and axis[-1] in 'XYZ':preferred['XYZ'.index(axis[-1])]=(-90. if axis.startswith('-') else 90.)
    result['preferred_angles']=preferred
    result.setdefault('use_preferred_angles',any(preferred))
    if 'limits' not in result:
        result['limits']=['Free']*3
        if result.get('is_hinge'):
            hinge=max(range(3),key=lambda i:abs(preferred[i]))
            result['limits']=['Free' if i==hinge else 'Locked' for i in range(3)]
    result.setdefault('minimum',[0.,0.,0.]);result.setdefault('maximum',[0.,0.,0.])
    return result

class FullBodyIKSolver(object):
    def __init__(self,solver_key,skeleton,char_profile,pose_mgr=None):
        self.solver_key=solver_key;self.skeleton=skeleton;self.char_profile=char_profile;self.pose_mgr=pose_mgr
        self._is_solving=False;self.goal_locators={}
        self.fbik_node=None;self.settings_group=None
        self.pelvis_dag=char_profile.get('pelvis_bone_dag') or skeleton.root_joint
        self.bone_chains=char_profile.get('bone_chains',{})
        self.fbik_settings=char_profile.get('fbik_bone_settings',{})
        self.solver_settings=dict(DEFAULT_SOLVER_SETTINGS);self.solver_settings.update(char_profile.get('fbik_solver_settings',{}))
        self.original_offsets={};self.bone_setting_nodes={}
        self.refresh_base_pose()
        self.fbik_settings=calibrate_knee_settings(skeleton,self.bone_chains,self.fbik_settings,self.base_world_pos,self.base_world_rot)
        char_profile['fbik_bone_settings']=self.fbik_settings

    def refresh_base_pose(self):
        self.base_world_pos={};self.base_world_rot={};self.base_local_pos={};self.base_local_rot={}
        for dag in self.skeleton.joint_names:
            p=self.pose_mgr.get_aligned_rest_position(dag) if self.pose_mgr else self.skeleton.rest_world_pos[dag]
            q=self.pose_mgr.get_aligned_rest_rotation(dag) if self.pose_mgr else self.skeleton.rest_world_rot[dag]
            self.base_world_pos[dag]=om.MVector(p);self.base_world_rot[dag]=om.MQuaternion(q)
        for i,dag in enumerate(self.skeleton.joint_names):
            parent=self.skeleton.parent_indices[i];p=self.base_world_pos[dag];q=self.base_world_rot[dag]
            if parent>=0:
                pd=self.skeleton.joint_names[parent];pq=self.base_world_rot[pd]
                p=rotate_vector_by_quat(p-self.base_world_pos[pd],pq.inverse());q=q*pq.inverse()
            self.base_local_pos[dag]=p;self.base_local_rot[dag]=q.normal()

    def register_goal_locator(self,name,locator_dag,bone_dag):
        p,q=get_world_transform(locator_dag)
        self.goal_locators[name]={'locator_dag':locator_dag,'bone_dag':bone_dag,'init_pos':p,'init_rot':q}

    def sync_locators_to_skeleton(self):
        """Keep the retarget processor's synchronization API for four Goals."""
        self._is_solving=True
        try:
            for info in self.goal_locators.values():
                locator=info['locator_dag'];bone=info['bone_dag']
                if cmds.objExists(locator) and cmds.objExists(bone):
                    p,q=get_world_transform(bone)
                    set_world_transform(locator,p,q)
        finally:
            self._is_solving=False

    @property
    def pelvis_locator_dag(self):
        """Compatibility with UI objects created before control removal."""
        return None

    def _solve_internal(self):
        if self.fbik_node and cmds.objExists(self.fbik_node):
            cmds.dgdirty(self.fbik_node)
            cmds.getAttr(self.fbik_node+'.outputOffset[0]')

    def solve_from_scene_goals(self):
        if not self._is_solving:self._solve_internal()

    def clear(self):
        if not self.fbik_node or not cmds.objExists(self.fbik_node):return
        for dag,matrix in self.original_offsets.items():
            destination=dag+'.offsetParentMatrix'
            sources=cmds.listConnections(destination,s=True,d=False,plugs=True) or []
            if sources and sources[0].split('.')[0]==self.fbik_node:
                cmds.disconnectAttr(sources[0],destination);cmds.setAttr(destination,*matrix,type='matrix')
        cmds.delete(self.fbik_node);self.fbik_node=None

def clear_fbik_callbacks(solver_key=None):
    """Remove owned DG outputs, or callbacks / expressions from the old version."""
    keys=[solver_key] if solver_key else list(set(_ACTIVE_CALLBACK_IDS)|set(_ACTIVE_FBIK_SOLVERS))
    import __main__
    for key in keys:
        for callback in _ACTIVE_CALLBACK_IDS.pop(key,[]):
            try:om.MMessage.removeCallback(callback)
            except RuntimeError:pass
        solver=_ACTIVE_FBIK_SOLVERS.pop(key,None)
        if solver and hasattr(solver,'clear'):solver.clear()
        expression='UE58_FBIK_Expr_'+key
        if cmds.objExists(expression):cmds.delete(expression)
        getattr(__main__,'_ue58_fbik_solvers',{}).pop(key,None)

def register_live_fbik_solver(solver,fbik_group_dag):
    key=solver.solver_key;clear_fbik_callbacks(key)
    library() # Build/load outside DG compute, before creating any output links.
    if 'ue58PBIKSolver' not in cmds.allNodeTypes():
        cmds.loadPlugin(str(Path(__file__).with_name('ue58_pbik_node.py')),quiet=True)
    skel=solver.skeleton
    for dag in skel.joint_names:
        if cmds.listConnections(dag+'.offsetParentMatrix',s=True,d=False):
            raise RuntimeError('FBIK requires an available offsetParentMatrix: '+dag)
        if cmds.getAttr(dag+'.offsetParentMatrix',lock=True):
            raise RuntimeError('FBIK cannot drive a locked offsetParentMatrix: '+dag)
    node=cmds.createNode('ue58PBIKSolver',name='UE58_PBIK_'+key,skipSelect=True)
    solver.fbik_node=node;solver.settings_group=fbik_group_dag
    cmds.setAttr(node+'.enabled',False)
    # UE's auto-generated rigs append effectors in this order. Constraint order
    # affects a finite-iteration PBIK solve, so preserve it in the adapter.
    order=['LeftFootIK','LeftHandIK','RightFootIK','RightHandIK']
    solver.goal_locators=dict(sorted(solver.goal_locators.items(),key=lambda item:order.index(item[0]) if item[0] in order else len(order)))
    root=skel.joint_indices[solver.pelvis_dag];goals=[]
    for name,info in solver.goal_locators.items():
        p=info['init_pos'];q=info['init_rot']
        goals.append({'name':name,'bone':skel.joint_indices[info['bone_dag']],
            'initial_position':[p.x,p.y,p.z],'initial_rotation':[q.x,q.y,q.z,q.w]})
    native_reference=[];alignment=[]
    for i,dag in enumerate(skel.joint_names):
        p=solver.base_world_pos[dag];q=solver.base_world_rot[dag]
        native_reference.append(to_engine_pose([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
        solver.original_offsets[dag]=list(cmds.getAttr(dag+'.offsetParentMatrix'))
        if solver.pose_mgr:
            delta=(skel.rest_local_rot[dag].inverse()*solver.base_local_rot[dag]).normal()
            alignment.append([delta.x,delta.y,delta.z,delta.w])
        else:alignment.append(None)
    cfg={'parents':skel.parent_indices,'root':root,'goals':goals,'native_reference':native_reference,
        'baseline_offsets':[solver.original_offsets[d] for d in skel.joint_names],
        'alignment_rotations':alignment}
    cmds.setAttr(node+'.configuration',json.dumps(cfg),type='string')
    for i,dag in enumerate(skel.joint_names):cmds.connectAttr(dag+'.matrix',node+'.inputLocalMatrix[{}]'.format(i))
    parents=cmds.listRelatives(skel.root_joint,parent=True,fullPath=True) or []
    if parents:cmds.connectAttr(parents[0]+'.worldMatrix[0]',node+'.externalParentMatrix')
    if cmds.objExists('time1'):cmds.connectAttr('time1.outTime',node+'.inputTime')
    for i,info in enumerate(solver.goal_locators.values()):
        loc=info['locator_dag'];cmds.connectAttr(loc+'.worldMatrix[0]',node+'.goalMatrix[{}]'.format(i))
        for name in ['positionAlpha','rotationAlpha','strengthAlpha','pullChainAlpha','pinRotation','chainDepth','previewMode']:
            cmds.connectAttr(loc+'.'+name,node+'.{}[{}]'.format(name,i))
    for name in ['iterations','subIterations','rootMode','allowStretch','massMultiplier','globalPullChainAlpha','maxAngle','overRelaxation']:
        cmds.connectAttr(fbik_group_dag+'.'+name,node+'.'+name)
    candidates=cmds.listRelatives(fbik_group_dag,children=True,fullPath=True) or []
    for clean,settings in solver.fbik_settings.items():
        dag=skel.resolve_bone_dag(clean)
        if not dag:continue
        i=skel.joint_indices[dag]
        bone_node=next((n for n in candidates if cmds.attributeQuery('boneDAG',node=n,exists=True) and cmds.getAttr(n+'.boneDAG')==dag),None)
        if not bone_node:continue
        solver.bone_setting_nodes[clean]=bone_node
        for source,target in [('rotationStiffness','boneRotationStiffness'),('positionStiffness','bonePositionStiffness'),
                ('limits','boneLimits'),('minimum','boneMinimum'),('maximum','boneMaximum'),
                ('preferredAngles','bonePreferredAngles'),('usePreferredAngles','boneUsePreferredAngles'),('isExcluded','boneExcluded')]:
            cmds.connectAttr(bone_node+'.'+source,node+'.{}[{}]'.format(target,i))
    for i,dag in enumerate(skel.joint_names):cmds.connectAttr(node+'.outputOffset[{}]'.format(i),dag+'.offsetParentMatrix')
    cmds.connectAttr(fbik_group_dag+'.enabled',node+'.enabled')
    _ACTIVE_FBIK_SOLVERS[key]=solver
    import __main__
    if not hasattr(__main__,'_ue58_fbik_solvers'):__main__._ue58_fbik_solvers={}
    __main__._ue58_fbik_solvers[key]=solver
    solver._solve_internal()
