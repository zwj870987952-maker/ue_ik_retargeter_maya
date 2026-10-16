"""Export isolated A/B .ma caches, retarget inside them, and bake back to B."""
import json
import math
import uuid
from pathlib import Path
from contextlib import contextmanager
import maya.cmds as cmds
import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
from .skeleton import RetargetSkeleton,resolve_skeleton_root_joint
from .processor import IKRetargetProcessor
from .retarget_assets import MayaSceneAssetBuilder
from .fbik_scene import clear_fbik_callbacks,_ACTIVE_FBIK_SOLVERS,library

CHANNELS=['translateX','translateY','translateZ','rotateX','rotateY','rotateZ','scaleX','scaleY','scaleZ']
CACHE_NAMESPACE='UE58RigCache'
SCHEMA=1

@contextmanager
def scene_state(undo_name=None,temporary=False):
    frame=cmds.currentTime(q=True);selection=cmds.ls(sl=True,long=True) or []
    auto=cmds.autoKeyframe(q=True,state=True);namespace=cmds.namespaceInfo(currentNamespace=True)
    undo=cmds.undoInfo(q=True,state=True);modified=cmds.file(q=True,modified=True)
    if temporary:cmds.undoInfo(stateWithoutFlush=False)
    elif undo_name:cmds.undoInfo(openChunk=True,chunkName=undo_name)
    cmds.autoKeyframe(state=False)
    failed=False
    try:yield
    except Exception:
        failed=True
        raise
    finally:
        cmds.namespace(set=namespace if cmds.namespace(exists=namespace) else ':')
        cmds.currentTime(frame,e=True)
        existing=[n for n in selection if cmds.objExists(n)]
        cmds.select(existing,r=True) if existing else cmds.select(clear=True)
        cmds.autoKeyframe(state=auto)
        if temporary:
            cmds.file(modified=modified)
            cmds.undoInfo(stateWithoutFlush=undo)
        elif undo_name:
            cmds.undoInfo(closeChunk=True)
            if failed and undo:cmds.undo()

def _string(node,name,value):
    if not cmds.attributeQuery(name,node=node,exists=True):cmds.addAttr(node,ln=name,dt='string')
    cmds.setAttr(node+'.'+name,value,type='string')

def _parent_matrix(root):
    p=cmds.listRelatives(root,parent=True,fullPath=True) or []
    return list(cmds.xform(p[0],q=True,ws=True,matrix=True)) if p else list(om.MMatrix())

def _baseline(joint):
    incoming=cmds.listConnections(joint+'.offsetParentMatrix',s=True,d=False,p=True) or []
    if incoming and cmds.nodeType(incoming[0].split('.')[0])=='ue58PBIKSolver':
        node=incoming[0].split('.')[0];cfg=json.loads(cmds.getAttr(node+'.configuration'))
        index=int(incoming[0].rsplit('[',1)[1][:-1])
        return list(cfg['baseline_offsets'][index])
    return list(cmds.getAttr(joint+'.offsetParentMatrix'))

def _records(root):
    skeleton=RetargetSkeleton(root);result=[]
    for i,j in enumerate(skeleton.joint_names):
        relative='/'.join(p.split(':')[-1] for p in j.split('|')[root.count('|'):])
        result.append({'index':i,'parent':skeleton.parent_indices[i],'id':relative,
            'name':j.split('|')[-1].split(':')[-1],'original_dag':j,
            'baseline':_baseline(j),'rotateOrder':cmds.getAttr(j+'.rotateOrder'),
            'segmentScaleCompensate':cmds.getAttr(j+'.segmentScaleCompensate'),
            'vectors':{a:list(cmds.getAttr(j+'.'+a)[0]) for a in ['translate','rotate','scale','jointOrient','rotateAxis','inverseScale']}})
    return result

def _clone(records,parent,namespace):
    joints=[]
    for r in records:
        j=cmds.createNode('joint',name=':'+namespace+':'+r['name'],parent=joints[r['parent']] if r['parent']>=0 else parent,skipSelect=True)
        for attr in ['rotateOrder','segmentScaleCompensate']:cmds.setAttr(j+'.'+attr,r[attr])
        for attr,values in r['vectors'].items():cmds.setAttr(j+'.'+attr,*values)
        cmds.setAttr(j+'.offsetParentMatrix',*r['baseline'],type='matrix')
        cmds.addAttr(j,ln='ue58CacheBoneIndex',at='long');cmds.setAttr(j+'.ue58CacheBoneIndex',r['index'])
        _string(j,'ue58OriginalBoneId',r['id'])
        joints.append(cmds.ls(j,long=True)[0])
    return joints

def _namespace_nodes(namespace):
    prefix=namespace.lstrip(':')+':'
    return [n for n in cmds.ls(long=True) if n.split('|')[-1].startswith(prefix)]

def _delete_namespace(namespace):
    cmds.namespace(set=':')
    nodes=_namespace_nodes(namespace)
    if nodes:cmds.delete(nodes)
    children=cmds.namespaceInfo(namespace,listOnlyNamespaces=True,recurse=True) or []
    for child in sorted(children,key=lambda n:n.count(':'),reverse=True):
        if cmds.namespace(exists=child):cmds.namespace(removeNamespace=child)
    if cmds.namespace(exists=namespace):cmds.namespace(removeNamespace=namespace)

def build_cache(source_root,target_root,output_dir=None,enable_ik=False,align_poles=False,pelvis_settings=None):
    """Temporarily clone raw input skeletons, export their rigs, then remove them."""
    source=resolve_skeleton_root_joint(source_root);target=resolve_skeleton_root_joint(target_root)
    if not source or not target or source==target:raise ValueError('Select two distinct source and target skeletons.')
    if cmds.namespace(exists=CACHE_NAMESPACE):raise RuntimeError('A cache build namespace already exists; finish its previous build first.')
    directory=Path(output_dir or (Path(cmds.file(q=True,sn=True)).parent/'retarget_assets')).resolve()
    directory.mkdir(parents=True,exist_ok=True)
    token=uuid.uuid4().hex[:8];stem='{}_to_{}_{}'.format(source.split('|')[-1].replace(':','_'),target.split('|')[-1].replace(':','_'),token)
    ma_path=directory/(stem+'.rig.ma');manifest_path=directory/(stem+'.rtg.json')
    records={'Source':_records(source),'Target':_records(target)}
    reference_parents={'Source':_parent_matrix(source),'Target':_parent_matrix(target)}
    keys=[]
    with scene_state(temporary=True):
        try:
            cmds.namespace(add=CACHE_NAMESPACE)
            owner=cmds.createNode('transform',name=':'+CACHE_NAMESPACE+':RigCache',skipSelect=True)
            cmds.setAttr(owner+'.visibility',False)
            sides={};rigs={}
            for role in ['Source','Target']:
                ns=CACHE_NAMESPACE+':'+role;cmds.namespace(add=role,parent=':'+CACHE_NAMESPACE)
                side=cmds.createNode('dagContainer',name=':'+CACHE_NAMESPACE+':'+role+'_FBIK',parent=owner,skipSelect=True)
                _string(side,'ue58CacheRole',role);sides[role]=side
                rigs[role]=_clone(records[role],side,ns)
            proc=IKRetargetProcessor(rigs['Source'][0],rigs['Target'][0])
            proc.enable_ik_pass=bool(enable_ik);proc.enable_pole_vector_op=bool(align_poles)
            proc.pelvis_solver.floor_constraint_weight=0.
            proc.pelvis_solver.affect_ik_vertical=0.
            for key,value in (pelvis_settings or {}).items():
                if hasattr(proc.pelvis_solver,key):setattr(proc.pelvis_solver,key,value)
            # Keep serialized reference frames fixed for the lifetime of the cache.
            bundle={'src_ikrig':proc.source_ikrig_asset.to_dict(),'tgt_ikrig':proc.target_ikrig_asset.to_dict(),'rtg':proc.rtg_asset.to_dict()}
            manifest={'schema':SCHEMA,'cache_ma':ma_path.name,'source_root':source,'target_root':target,
                'reference_frame':cmds.currentTime(q=True),'records':records,'reference_parent_matrices':reference_parents,
                'bundle':bundle,'time_unit':cmds.currentUnit(q=True,time=True),'linear_unit':cmds.currentUnit(q=True,linear=True)}
            for role,asset,pose,color in [('Source',proc.source_ikrig_asset,None,(.2,.8,1.)),('Target',proc.target_ikrig_asset,proc.target_pose_mgr,(.2,1.,.4))]:
                ns=CACHE_NAMESPACE+':'+role;cmds.namespace(set=':'+ns);key='CacheBuild_'+token+'_'+role;keys.append(key)
                _,solver=MayaSceneAssetBuilder._build_ikrig_scene_node(sides[role],asset,key,pose,'',color,color)
                _string(solver.fbik_node,'ue58CacheRole',role)
                if role=='Source':
                    driver=cmds.listConnections(solver.fbik_node+'.enabled',s=True,d=False,p=True)[0]
                    cmds.setAttr(driver,False)
                members=[n for n in _namespace_nodes(ns) if not cmds.objectType(n,isAType='dagNode')]
                if members:cmds.container(sides[role],e=True,addNode=members,force=True)
            cmds.namespace(set=':'+CACHE_NAMESPACE)
            _string(owner,'ue58RigCacheManifest',json.dumps(manifest,ensure_ascii=False))
            cmds.select(_namespace_nodes(CACHE_NAMESPACE),r=True)
            cmds.file(str(ma_path),force=True,options='v=0;',type='mayaAscii',exportSelected=True,preserveReferences=False)
            manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        finally:
            for key in keys:clear_fbik_callbacks(key)
            cmds.namespace(set=':')
            if cmds.namespace(exists=CACHE_NAMESPACE):_delete_namespace(CACHE_NAMESPACE)
    return {'ma':str(ma_path),'rtg':str(manifest_path),'manifest':manifest}

def _original_joints(root,records):
    skeleton=RetargetSkeleton(root)
    by_id={'/'.join(p.split(':')[-1] for p in j.split('|')[root.count('|'):]):j for j in skeleton.joint_names}
    result=[]
    for r in records:
        if r['id'] not in by_id:raise ValueError('Cached bone is missing from the current skeleton: '+r['id'])
        result.append(by_id[r['id']])
    return result

def _cached_joints(side):
    nodes=cmds.listRelatives(side,ad=True,type='joint',fullPath=True) or []
    return sorted((n for n in nodes if cmds.attributeQuery('ue58CacheBoneIndex',node=n,exists=True)),key=lambda n:cmds.getAttr(n+'.ue58CacheBoneIndex'))

def _rig_order(rig,joints):
    """Rig traversal can differ from creation order; bind by stable bone IDs."""
    by_id={cmds.getAttr(j+'.ue58OriginalBoneId'):j for j in joints}
    bones=rig['skeleton']['bones'];root=bones[0]['dag_path']
    ids=['/'.join(p.split(':')[-1] for p in b['dag_path'].split('|')[root.count('|'):]) for b in bones]
    return [by_id[key] for key in ids]

def cache_with_ue_profile(cache_path,profile_path,output_dir=None):
    """Save a new .ma/JSON cache using an exported profile of matching UE rigs."""
    from .ue_reference import apply_ue_profile
    from .pose_evaluator import CachedPoseEvaluator
    from .math_utils import trs_to_matrix
    from .pbik_native import to_engine_pose
    path=Path(cache_path).resolve()
    manifest=json.loads(path.read_text(encoding='utf-8')) if path.suffix.lower()!='.ma' else None
    ma_path=path if manifest is None else path.parent/manifest['cache_ma']
    profile=json.loads(Path(profile_path).read_text(encoding='utf-8'))
    directory=Path(output_dir or path.parent).resolve();directory.mkdir(parents=True,exist_ok=True)
    token=uuid.uuid4().hex[:8];stem=ma_path.name.removesuffix('.rig.ma')+'_UE_'+token
    output_ma=directory/(stem+'.rig.ma');output_json=directory/(stem+'.rtg.json')
    namespace='UE58CacheProfile_'+token
    if cmds.namespace(exists=CACHE_NAMESPACE):raise ValueError('A cache build is already active.')
    relocated=False
    with scene_state(temporary=True):
        unit=cmds.currentUnit(q=True,time=True);linear=cmds.currentUnit(q=True,linear=True)
        try:
            cmds.file(str(ma_path),i=True,type='mayaAscii',namespace=namespace,mergeNamespacesOnClash=False)
            cmds.currentUnit(time=unit,linear=linear,updateAnimation=False)
            candidates=[n for n in _namespace_nodes(namespace) if cmds.attributeQuery('ue58RigCacheManifest',node=n,exists=True)]
            if len(candidates)!=1:raise ValueError('Expected one owned cache root.')
            owner=candidates[0]
            if manifest is None:manifest=json.loads(cmds.getAttr(owner+'.ue58RigCacheManifest'))
            manifest['bundle']=apply_ue_profile(manifest['bundle'],profile);manifest['cache_ma']=output_ma.name
            nodes={cmds.getAttr(n+'.ue58CacheRole'):n for n in _namespace_nodes(namespace) if cmds.nodeType(n)=='ue58PBIKSolver'}
            sides={cmds.getAttr(n+'.ue58CacheRole'):n for n in _namespace_nodes(namespace) if cmds.nodeType(n)=='dagContainer' and cmds.attributeQuery('ue58CacheRole',node=n,exists=True)}
            evaluator=CachedPoseEvaluator(manifest['bundle'],{})
            for role,key,pose in [('Source','src_ikrig',evaluator.source_pose),('Target','tgt_ikrig',evaluator.target_pose)]:
                rig=manifest['bundle'][key];ordered=_rig_order(rig,_cached_joints(sides[role]))
                node=nodes[role];config=json.loads(cmds.getAttr(node+'.configuration'))
                driver=cmds.listConnections(node+'.enabled',s=True,d=False,p=True)[0]
                cmds.setAttr(driver,False)
                for i,(bone,joint) in enumerate(zip(rig['skeleton']['bones'],ordered)):
                    # Place the cached input in UE's mesh reference pose, while
                    # keeping Maya jointOrient/rotateAxis as imported.
                    cmds.xform(joint,ws=True,matrix=list(trs_to_matrix(om.MVector(*bone['ref_world_pos']),om.MQuaternion(*bone['ref_world_rot']))))
                config['native_reference']=[to_engine_pose(b['ref_world_pos'],b['ref_world_rot']) for b in rig['skeleton']['bones']]
                config['alignment_rotations']=[]
                for bone in rig['skeleton']['bones']:
                    delta=(om.MQuaternion(*bone['ref_local_rot']).inverse()*pose.aligned_local_rot[bone['dag_path']]).normal()
                    config['alignment_rotations'].append(list(delta) if role=='Target' else None)
                for i,g in enumerate(config['goals']):
                    name=rig['skeleton']['bones'][g['bone']]['dag_path']
                    p=pose.get_aligned_rest_position(name);q=pose.get_aligned_rest_rotation(name)
                    g['initial_position']=list(p);g['initial_rotation']=list(q)
                    control=cmds.listConnections(node+'.goalMatrix[{}]'.format(i),s=True,d=False)[0]
                    cmds.xform(control,ws=True,matrix=list(trs_to_matrix(p,q)))
                cmds.setAttr(node+'.configuration',json.dumps(config),type='string')
                if role=='Target':cmds.setAttr(driver,True)
            evaluator.close()
            _string(owner,'ue58RigCacheManifest',json.dumps(manifest,ensure_ascii=False))
            # Remove the temporary import prefix so repeated imports have the
            # same portable namespace layout as an ordinary generated cache.
            cmds.namespace(set=':')
            cmds.namespace(removeNamespace=':'+namespace,mergeNamespaceWithRoot=True)
            relocated=True
            cmds.select(_namespace_nodes(CACHE_NAMESPACE),r=True)
            cmds.file(str(output_ma),force=True,options='v=0;',type='mayaAscii',exportSelected=True,preserveReferences=False)
            output_json.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        finally:
            cmds.currentUnit(time=unit,linear=linear,updateAnimation=False)
            for ns in ([namespace,CACHE_NAMESPACE] if relocated else [namespace]):
                if cmds.namespace(exists=ns):
                    for node in _namespace_nodes(ns):
                        if cmds.nodeType(node)=='ue58PBIKSolver':_close_node(node)
                    _delete_namespace(ns)
    return {'ma':str(output_ma),'rtg':str(output_json),'manifest':manifest}

def _close_node(node):
    selection=om.MSelectionList();selection.add(node)
    instance=om.MFnDependencyNode(selection.getDependNode(0)).userNode()
    runtime=getattr(instance,'retarget_runtime',None)
    if runtime:runtime.close();instance.retarget_runtime=None
    instance.retarget_signature=None
    native=getattr(instance,'native',None)
    if native:native.close();instance.native=None
    instance.signature=None

def remove_cached_systems(owner):
    if not cmds.objExists(owner) or not cmds.attributeQuery('ue58SessionNamespace',node=owner,exists=True):
        raise ValueError('This is not an owned retarget cache session.')
    namespace=cmds.getAttr(owner+'.ue58SessionNamespace')
    for n in _namespace_nodes(namespace):
        if cmds.nodeType(n)=='ue58PBIKSolver':_close_node(n)
    with scene_state(undo_name='Remove cached A B systems'):_delete_namespace(namespace)

def _detach_legacy_outputs(joints):
    """Retire only this project's older live rigs attached to original actors."""
    nodes={p.split('.')[0] for j in joints for p in (cmds.listConnections(j+'.offsetParentMatrix',s=True,d=False,p=True) or []) if cmds.nodeType(p.split('.')[0])=='ue58PBIKSolver'}
    retired_assets=set()
    for node in nodes:
        cfg=json.loads(cmds.getAttr(node+'.configuration'))
        incoming=[]
        for i,m in enumerate(cfg['baseline_offsets']):
            source=cmds.listConnections(node+'.inputLocalMatrix[{}]'.format(i),s=True,d=False,p=True) or []
            if source:
                joint=cmds.ls(source[0].rsplit('.matrix',1)[0],long=True)[0]
                incoming.append((joint,m,i))
        if any(j not in joints for j,_,_ in incoming):raise ValueError('A live rig also owns unrelated joints: '+node)
        setting_plug=cmds.listConnections(node+'.enabled',s=True,d=False,p=True) or []
        asset=None
        if setting_plug:
            parents=cmds.listRelatives(setting_plug[0].split('.')[0],parent=True,fullPath=True) or []
            if parents and cmds.attributeQuery('ueAssetType',node=parents[0],exists=True):asset=parents[0]
        for j,m,i in incoming:
            output=node+'.outputOffset[{}]'.format(i)
            if cmds.isConnected(output,j+'.offsetParentMatrix'):cmds.disconnectAttr(output,j+'.offsetParentMatrix');cmds.setAttr(j+'.offsetParentMatrix',*m,type='matrix')
        for key,solver in list(_ACTIVE_FBIK_SOLVERS.items()):
            if solver.fbik_node==node:solver.fbik_node=None;clear_fbik_callbacks(key)
        _close_node(node);cmds.delete(node)
        if asset and cmds.objExists(asset):cmds.delete(asset)

        if asset:retired_assets.add(asset.split('|')[-1])
    top='|'+MayaSceneAssetBuilder.CONTAINER_GRP
    if retired_assets and cmds.objExists(top):
        for child in cmds.listRelatives(top,c=True,fullPath=True) or []:
            if not all(cmds.attributeQuery(a,node=child,exists=True) for a in ['sourceIKRigAsset','targetIKRigAsset']):continue
            pair={cmds.getAttr(child+'.sourceIKRigAsset'),cmds.getAttr(child+'.targetIKRigAsset')}
            if pair<=retired_assets:cmds.delete(child)
        if not cmds.listRelatives(top,c=True):cmds.delete(top)

def _write_curves(joints,frames,values,start,end):
    """New independent curves make the output removable from its cache rig.

    Preserve old target curves and keys outside the requested range by copying
    them before replacing the interval. New nodes/connections remain undoable.
    """
    cmds.namespace(set=':')
    curves=[];time_array=om.MTimeArray([om.MTime(t,om.MTime.uiUnit()) for t in frames])
    for index,joint in enumerate(joints):
        previous=None;order=cmds.getAttr(joint+'.rotateOrder')
        for sample in values:
            euler=om.MEulerRotation(*[v*math.pi/180. for v in sample[index][3:6]],order)
            if previous is not None:euler=euler.closestSolution(previous)
            sample[index][3:6]=[v*180./math.pi for v in euler];previous=euler
    for index,joint in enumerate(joints):
        for channel,attr in enumerate(CHANNELS):
            plug=joint+'.'+attr
            if cmds.getAttr(plug,lock=True):raise ValueError('Locked animation channel: '+plug)
            old=cmds.listConnections(plug,s=True,d=False,p=True) or []
            if old and not cmds.nodeType(old[0].split('.')[0]).startswith('animCurve'):
                raise ValueError('Animation layers or constraints require an available output skeleton: '+plug)
            curve_type='animCurveTA' if attr.startswith('rotate') else ('animCurveTL' if attr.startswith('translate') else 'animCurveTU')
            name=joint.split('|')[-1]+'_Retarget_'+attr
            curve=cmds.createNode(curve_type,name=':'+name,skipSelect=True)
            incoming=cmds.listConnections(curve+'.input',s=True,d=False,p=True) or []
            if not incoming:cmds.connectAttr(':time1.outTime',curve+'.input')
            selection=om.MSelectionList();selection.add(curve);fn=oma.MFnAnimCurve(selection.getDependNode(0))
            if old:
                old_selection=om.MSelectionList();old_selection.add(old[0].split('.')[0])
                old_fn=oma.MFnAnimCurve(old_selection.getDependNode(0))
                fn.setIsWeighted(old_fn.isWeighted)
                fn.setPreInfinityType(old_fn.preInfinityType);fn.setPostInfinityType(old_fn.postInfinityType)
                copied=[]
                for key in range(old_fn.numKeys):
                    key_time=old_fn.input(key)
                    if key_time.asUnits(om.MTime.uiUnit())<start or key_time.asUnits(om.MTime.uiUnit())>end:
                        new_index=fn.addKey(key_time,old_fn.value(key),old_fn.inTangentType(key),old_fn.outTangentType(key))
                        copied.append((key,new_index))
                # API angle/weight copying retains fixed and weighted handles
                # outside the bake interval instead of recalculating their shape.
                for key,new_index in copied:
                    fn.setTangentsLocked(new_index,False);fn.setWeightsLocked(new_index,False)
                    for incoming in [True,False]:
                        if (old_fn.inTangentType(key) if incoming else old_fn.outTangentType(key))==oma.MFnAnimCurve.kTangentFixed:
                            angle,weight=old_fn.getTangentAngleWeight(key,incoming)
                            fn.setTangent(new_index,angle,weight,incoming)
                    fn.setWeightsLocked(new_index,old_fn.weightsLocked(key));fn.setTangentsLocked(new_index,old_fn.tangentsLocked(key))
            samples=[pose[index][channel]*(math.pi/180. if attr.startswith('rotate') else 1.) for pose in values]
            fn.addKeys(time_array,om.MDoubleArray(samples),oma.MFnAnimCurve.kTangentLinear,oma.MFnAnimCurve.kTangentLinear,True)
            cmds.connectAttr(curve+'.output',plug,force=True);curves.append(curve)
            if old and not cmds.listConnections(old[0].split('.')[0]+'.output',s=False,d=True):cmds.delete(old[0].split('.')[0])
    return curves

def _receiver(records,owner,namespace):
    group=cmds.createNode('transform',name=':'+namespace+':Receiver',parent=owner,skipSelect=True)
    cmds.setAttr(group+'.visibility',False)
    return group,_clone(records,group,namespace)

def _sample_input(joints,records,frame,reference_parent):
    world=[];parent_world=om.MMatrix(_parent_matrix(joints[0]));reference_inverse=om.MMatrix(reference_parent).inverse()
    for j,r in zip(joints,records):
        local=om.MMatrix(cmds.getAttr(j+'.matrix'))*om.MMatrix(_baseline(j))
        parent=r['parent'];world.append(local*(world[parent] if parent>=0 else parent_world))
    return [m*reference_inverse for m in world]

def _receive_pose(receiver,matrices,parent_matrix):
    for j,m in zip(receiver,matrices):cmds.xform(j,ws=True,matrix=list(m*parent_matrix))
    return [[cmds.getAttr(j+'.'+attr) for attr in CHANNELS] for j in receiver]

def _bake_output(owner,target,sides,cached,manifest,node,frames,start,end,progress):
    namespace=cmds.getAttr(owner+'.ue58SessionNamespace')
    cmds.namespace(set=':'+namespace)
    cmds.namespace(add='Receiver',parent=':'+namespace)
    group,receiver=_receiver(manifest['records']['Target'],owner,namespace+':Receiver')
    cmds.xform(group,ws=True,matrix=_parent_matrix(target))
    samples=[];maximum_native_solves=0
    selection=om.MSelectionList();selection.add(node)
    instance=om.MFnDependencyNode(selection.getDependNode(0)).userNode()
    for i,t in enumerate(frames):
        cmds.currentTime(t,e=True)
        display=om.MMatrix(cmds.xform(sides['Target'],q=True,ws=True,matrix=True)).inverse()
        output=[om.MMatrix(cmds.xform(j,q=True,ws=True,matrix=True))*display for j in cached['Target']]
        samples.append(_receive_pose(receiver,output,om.MMatrix(_parent_matrix(target))))
        maximum_native_solves=max(maximum_native_solves,instance.retarget_runtime.last_native_solves)
        if progress:progress('Retarget and bake B',i,len(frames))
    cmds.delete(group);cmds.namespace(removeNamespace=':'+namespace+':Receiver')
    original=_original_joints(target,manifest['records']['Target'])
    curves=_write_curves(original,frames,samples,start,end)
    _string(owner,'bakedRange',json.dumps([start,end]))
    return curves,maximum_native_solves

def bake_cached_output(owner,start=None,end=None,progress=None):
    """Bake a retained system after editing its goals or RTG configuration."""
    if not cmds.objExists(owner) or not cmds.attributeQuery('ue58SessionNamespace',node=owner,exists=True):
        raise ValueError('Select an owned retarget cache session.')
    manifest=json.loads(cmds.getAttr(owner+'.ue58RigCacheManifest'))
    target=resolve_skeleton_root_joint(cmds.getAttr(owner+'.targetOriginalRoot'))
    if not target:raise ValueError('The original target skeleton is missing.')
    if start is None or end is None:start,end=json.loads(cmds.getAttr(owner+'.bakedRange'))
    if end<start:raise ValueError('Invalid frame range.')
    namespace=cmds.getAttr(owner+'.ue58SessionNamespace')
    sides={cmds.getAttr(n+'.ue58CacheRole'):n for n in _namespace_nodes(namespace)
        if cmds.nodeType(n)=='dagContainer' and cmds.attributeQuery('ue58CacheRole',node=n,exists=True)}
    cached={r:_cached_joints(s) for r,s in sides.items()}
    node=next(n for n in _namespace_nodes(namespace) if cmds.nodeType(n)=='ue58PBIKSolver' and cmds.getAttr(n+'.ue58CacheRole')=='Target')
    frames=[float(t) for t in range(int(math.floor(start)),int(math.ceil(end))+1)]
    with scene_state(undo_name='Bake retained cached B output'):
        curves,solves=_bake_output(owner,target,sides,cached,manifest,node,frames,start,end,progress)
    return {'frames':len(frames),'target_curve_count':len(curves),'maximum_target_native_solves_per_evaluation':solves}

def retarget_from_cache(cache_path,source_root=None,target_root=None,start=None,end=None,keep_systems=True,display_offset=220.,progress=None,enable_ik=None,align_poles=None,pelvis_settings=None):
    """Import cached rigs, bake A's input into its cache, and bake B's output."""
    path=Path(cache_path).resolve()
    if path.suffix.lower()=='.ma':
        manifest=None;ma_path=path
    else:
        manifest=json.loads(path.read_text(encoding='utf-8'));ma_path=path.parent/manifest['cache_ma']
    if not ma_path.is_file():raise FileNotFoundError(str(ma_path))
    if manifest and manifest.get('schema')!=SCHEMA:raise ValueError('Unsupported rig cache schema.')
    namespace='UE58Session_'+uuid.uuid4().hex[:8];owner=None
    library()
    try:
        with scene_state(undo_name='Retarget animation through cached A B systems'):
            unit=cmds.currentUnit(q=True,time=True);linear=cmds.currentUnit(q=True,linear=True)
            cmds.file(str(ma_path),i=True,type='mayaAscii',namespace=namespace,mergeNamespacesOnClash=False,returnNewNodes=True)
            cmds.currentUnit(time=unit,linear=linear,updateAnimation=False)
            candidates=[n for n in _namespace_nodes(namespace) if cmds.attributeQuery('ue58RigCacheManifest',node=n,exists=True)]
            if len(candidates)!=1:raise ValueError('The .ma file does not contain one A/B rig cache.')
            owner=candidates[0]
            if manifest is None:manifest=json.loads(cmds.getAttr(owner+'.ue58RigCacheManifest'))
            if manifest.get('schema')!=SCHEMA:raise ValueError('Unsupported embedded rig cache schema.')
            _string(owner,'ue58RigCacheManifest',json.dumps(manifest,ensure_ascii=False))
            _string(owner,'ue58SessionNamespace',namespace);cmds.setAttr(owner+'.visibility',True)
            source=resolve_skeleton_root_joint(source_root or manifest['source_root']);target=resolve_skeleton_root_joint(target_root or manifest['target_root'])
            if not source or not target or source==target:raise ValueError('The original A/B skeletons are missing.')
            original={r:_original_joints(root,manifest['records'][r]) for r,root in [('Source',source),('Target',target)]}
            # Preflight before changing original output connections or keys.
            for j in original['Target']:
                opm_input=cmds.listConnections(j+'.offsetParentMatrix',s=True,d=False) or []
                if opm_input and cmds.nodeType(opm_input[0])!='ue58PBIKSolver':raise ValueError('Occupied target offsetParentMatrix: '+j)
                for attr in CHANNELS:
                    if cmds.getAttr(j+'.'+attr,lock=True):raise ValueError('Locked target channel: '+j+'.'+attr)
                    input_=cmds.listConnections(j+'.'+attr,s=True,d=False,p=True) or []
                    if input_ and not cmds.nodeType(input_[0].split('.')[0]).startswith('animCurve'):raise ValueError('Occupied target channel: '+j+'.'+attr)
            if start is None:start=cmds.playbackOptions(q=True,min=True)
            if end is None:end=cmds.playbackOptions(q=True,max=True)
            if end<start:raise ValueError('Invalid frame range.')
            frames=[float(t) for t in range(int(math.floor(start)),int(math.ceil(end))+1)]
            sides={cmds.getAttr(n+'.ue58CacheRole'):n for n in _namespace_nodes(namespace) if cmds.nodeType(n)=='dagContainer' and cmds.attributeQuery('ue58CacheRole',node=n,exists=True)}
            cached={r:_cached_joints(sides[r]) for r in sides}
            nodes={cmds.getAttr(n+'.ue58CacheRole'):n for n in _namespace_nodes(namespace) if cmds.nodeType(n)=='ue58PBIKSolver'}
            # UE reads the source animation directly. The retained source rig
            # remains editable; users can enable its IK explicitly afterwards.
            source_enable=cmds.listConnections(nodes['Source']+'.enabled',s=True,d=False,p=True) or []
            cmds.setAttr(source_enable[0] if source_enable else nodes['Source']+'.enabled',False)
            cmds.setAttr(sides['Source']+'.tx',display_offset)
            cmds.setAttr(sides['Target']+'.tx',display_offset*2.)
            _string(owner,'sourceOriginalRoot',source);_string(owner,'targetOriginalRoot',target)
            _detach_legacy_outputs(original['Source']+original['Target'])
            cmds.namespace(set=':'+namespace)
            cmds.namespace(add='Receiver',parent=':'+namespace)
            group,receiver=_receiver(manifest['records']['Source'],owner,namespace+':Receiver')
            input_samples=[]
            for i,t in enumerate(frames):
                cmds.currentTime(t,e=True)
                matrices=_sample_input(original['Source'],manifest['records']['Source'],t,manifest['reference_parent_matrices']['Source'])
                input_samples.append(_receive_pose(receiver,matrices,om.MMatrix(cmds.xform(owner,q=True,ws=True,matrix=True))))
                if progress:progress('Copy A animation',i,len(frames))
            cmds.delete(group);cmds.namespace(removeNamespace=':'+namespace+':Receiver')
            source_curves=_write_curves(cached['Source'],frames,input_samples,start,end)
            cmds.container(sides['Source'],e=True,addNode=source_curves,force=True)
            target_node=nodes['Target'];cfg=json.loads(cmds.getAttr(target_node+'.configuration'))
            for i in range(len(cached['Target'])):
                plug=target_node+'.inputLocalMatrix[{}]'.format(i)
                for connection in cmds.listConnections(plug,s=True,d=False,p=True) or []:cmds.disconnectAttr(connection,plug)
            cfg['retarget_bundle']=manifest['bundle'];cfg['source_count']=len(cached['Source'])
            ordered_source=_rig_order(manifest['bundle']['src_ikrig'],cached['Source'])
            ordered_target=_rig_order(manifest['bundle']['tgt_ikrig'],cached['Target'])
            for op in cfg['retarget_bundle']['rtg']['retarget_ops']:
                if enable_ik is not None and op['op_type'] in ['FIKRetargetIKChainsOp','FIKRetargetRunIKRigOp']:op['enabled']=bool(enable_ik)
                if align_poles is not None and op['op_type']=='FIKRetargetAlignPoleVectorOp':op['enabled']=bool(align_poles)
                if pelvis_settings and op['op_type']=='FIKRetargetPelvisMotionOp':op['settings'].update(pelvis_settings)
            cfg['align_poles']=next((op['enabled'] for op in cfg['retarget_bundle']['rtg']['retarget_ops'] if op['op_type']=='FIKRetargetAlignPoleVectorOp'),False)
            for i,j in enumerate(ordered_source):cmds.connectAttr(j+'.worldMatrix[0]',target_node+'.inputLocalMatrix[{}]'.format(i),force=True)
            for i,j in enumerate(ordered_target):cmds.connectAttr(j+'.matrix',target_node+'.inputLocalMatrix[{}]'.format(len(cached['Source'])+i),force=True)
            cmds.connectAttr(sides['Source']+'.worldMatrix[0]',target_node+'.goalMatrix[{}]'.format(len(cfg['goals'])),force=True)
            cmds.setAttr(target_node+'.configuration',json.dumps(cfg),type='string')
            # Bake the solved world pose through an unconnected receiver, then
            # create ordinary curves on original B under its own parent frame.
            target_curves,maximum_native_solves=_bake_output(owner,target,sides,cached,manifest,target_node,frames,start,end,progress)
            _string(owner,'cacheFile',str(ma_path))
            result={'session_root':owner,'namespace':namespace,'systems':sides,'source_root':source,'target_root':target,
                'range':[start,end],'frames':len(frames),'source_curve_count':len(source_curves),'target_curve_count':len(target_curves),
                'maximum_target_native_solves_per_evaluation':maximum_native_solves,'target_node':target_node,'source_node':nodes['Source'],
                'retained':bool(keep_systems)}
            if not keep_systems:remove_cached_systems(owner)
            return result
    except Exception:
        # File import is not fully undoable in Maya. Remove its namespace AFTER
        # rolling back channel/connection edits, so undo cannot resurrect it.
        with scene_state(temporary=True):
            if cmds.namespace(exists=namespace):
                for node in _namespace_nodes(namespace):
                    if cmds.nodeType(node)=='ue58PBIKSolver':_close_node(node)
                _delete_namespace(namespace)
        raise
