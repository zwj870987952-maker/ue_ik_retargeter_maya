"""Persistent Maya rig lifecycle, source cache and integer-frame curve authoring."""
import base64
from contextlib import contextmanager
import json
import math
from pathlib import Path
import uuid
import zlib
import maya.cmds as cmds
import maya.api.OpenMaya as om
from . import engine

SCHEMA=1
ATTRS=['translateX','translateY','translateZ','rotateX','rotateY','rotateZ']
ROLES=[('L_Foot','左脚','LeftFootIK'),('L_Hand','左手','LeftHandIK'),
       ('R_Foot','右脚','RightFootIK'),('R_Hand','右手','RightHandIK'),
       ('COM','质心 / 骨盆',''),('Chest','胸部',''),('Head','头部','')]
DEFAULT_ROLES=['L_Foot','L_Hand','R_Foot','R_Hand','COM']
_DEPTH=0

@contextmanager
def operation(name,mutate=True):
    global _DEPTH
    outer=_DEPTH==0;_DEPTH+=1
    if outer:
        frame=cmds.currentTime(q=True);selection=cmds.ls(sl=True,long=True) or []
        auto=cmds.autoKeyframe(q=True,state=True);ns=cmds.namespaceInfo(currentNamespace=True)
        undo=cmds.undoInfo(q=True,state=True)
        cmds.autoKeyframe(state=False)
        if mutate:
            if not undo:cmds.undoInfo(stateWithoutFlush=True)
            cmds.undoInfo(openChunk=True,chunkName=name)
        else:cmds.undoInfo(stateWithoutFlush=False)
    failed=False
    try:yield
    except Exception:failed=True;raise
    finally:
        _DEPTH-=1
        if outer:
            cmds.namespace(set=ns if cmds.namespace(exists=ns) else ':')
            cmds.currentTime(frame)
            alive=[n for n in selection if cmds.objExists(n)]
            cmds.select(alive,r=True) if alive else cmds.select(clear=True)
            cmds.autoKeyframe(state=auto)
            if mutate:
                cmds.undoInfo(closeChunk=True)
                if failed:cmds.undo()
                if not undo:cmds.undoInfo(stateWithoutFlush=False)
            else:cmds.undoInfo(stateWithoutFlush=undo)

def string(node,attr,value):
    if not cmds.attributeQuery(attr,node=node,exists=True):cmds.addAttr(node,ln=attr,dt='string')
    cmds.setAttr(node+'.'+attr,value,type='string')

def _pack(value):return base64.b64encode(zlib.compress(json.dumps(value,separators=(',',':')).encode('utf-8'))).decode('ascii')
def _unpack(value):return json.loads(zlib.decompress(base64.b64decode(value)).decode('utf-8'))
def uid(node):return cmds.ls(node,uuid=True)[0]
def resolve(identifier):
    values=cmds.ls(identifier,long=True) or []
    if not values:raise RuntimeError('节点已不存在：'+str(identifier))
    return values[0]

def roots():return [n for n in cmds.ls('*.awState',objectsOnly=True,long=True) or [] if cmds.objExists(n)]
def state(root):return json.loads(cmds.getAttr(root+'.awState'))
def store(root,value):string(root,'awState',json.dumps(value,ensure_ascii=False))
def source_cache(root):return _unpack(cmds.getAttr(root+'.awSourceCache'))
def controller(root,role):return resolve(state(root)['controls'][role])
def joints(root):
    s=state(root)
    return [resolve(i) for i in s['joints']]

def own(root,node):
    if node==root:return
    if not cmds.attributeQuery('awOwner',node=node,exists=True):cmds.addAttr(node,ln='awOwner',at='message')
    existing=cmds.listConnections(node+'.awOwner',s=True,d=False) or []
    if existing and uid(existing[0])!=uid(root):raise RuntimeError('节点属于另一套工具绑定：'+node)
    if not existing:cmds.connectAttr(root+'.message',node+'.awOwner')
    if not cmds.attributeQuery('awNodes',node=root,exists=True):cmds.addAttr(root,ln='awNodes',at='message',multi=True)
    members=cmds.listConnections(root+'.awNodes',s=True,d=False) or []
    if uid(node) not in {uid(n) for n in members}:
        indices=cmds.getAttr(root+'.awNodes',multiIndices=True) or []
        cmds.connectAttr(node+'.message',root+'.awNodes[{}]'.format(max(indices)+1 if indices else 0))

def owned(root):return list(dict.fromkeys(cmds.listConnections(root+'.awNodes',s=True,d=False) or [])) if cmds.attributeQuery('awNodes',node=root,exists=True) else []

def _vec_attr(node,name,kind,default,**kwargs):
    cmds.addAttr(node,ln=name,at=kind,dv=default,**kwargs)

def _quaternion_keys(curves):
    # Filtering alone does not fix intermediate layer weights. Both settings matter.
    if not all(cmds.getAttr(c+'.rotationInterpolation')>=3 for c in curves):cmds.filterCurve(curves)
    cmds.rotationInterpolation(curves,convert='quaternionSlerp')

def eulers(quaternions,order=0):
    values=[];previous=None
    for q in quaternions:
        e=om.MQuaternion(*q).asEulerRotation();e.reorderIt(order)
        if previous is not None:e=e.closestSolution(previous)
        values.append([math.degrees(v) for v in e]);previous=e
    return values

def layer_curve(layer,node,attribute):
    value=cmds.animLayer(layer,q=True,findCurveForPlug=node+'.'+attribute)
    return value[0] if isinstance(value,(list,tuple)) else value

def ensure_layer(root,name,controls):
    if not cmds.objExists(name):name=cmds.animLayer(name,override=True)
    own(root,name);cmds.setAttr(name+'.rotationAccumulationMode',1)
    cmds.animLayer(name,e=True,attribute=[c+'.'+a for c in controls for a in ATTRS])
    return name

def write_pose(root,node,times,positions,rotations,layer=None):
    order=cmds.getAttr(node+'.rotateOrder');angles=eulers(rotations,order)
    rows=[p+angles[i] for i,p in enumerate(positions)];curves={}
    target_layer=layer or ('BaseAnimation' if cmds.objExists('BaseAnimation') else None)
    def find(attr):
        curve=layer_curve(target_layer,node,attr) if target_layer else None
        # A scene may have BaseAnimation while this plug has never joined any
        # layer. Maya still keys it directly, but findCurveForPlug returns None.
        if not curve and (not layer or target_layer=='BaseAnimation'):
            curve=(cmds.listConnections(node+'.'+attr,s=True,d=False,type='animCurve') or [None])[0]
        return curve
    missing=[a for a in ATTRS if not find(a)]
    if missing:
        # Create all missing channels BEFORE authoring any values. Keying one
        # rotation plug through a By-Layer quaternion blend can key/reset its
        # sibling axes at the same time, especially on repeated regeneration.
        kwargs=dict(attribute=missing,time=times[0])
        if target_layer:kwargs['animLayer']=target_layer
        cmds.setKeyframe(node,**kwargs)
    # Maya synchronizes sibling axes when setKeyframe targets a quaternion
    # curve (synchronized Euler mode does this too). Write independent Euler
    # channels in "none" authoring mode, then convert their
    # completed curves together back to shortest-path quaternion playback.
    rotation_curves=[find(a) for a in ATTRS[3:]]
    if all(rotation_curves):cmds.rotationInterpolation(rotation_curves,convert='none')
    for k,attr in enumerate(ATTRS):
        curve=find(attr)
        if not curve:raise RuntimeError('无法获取动画曲线：'+node+'.'+attr)
        own(root,curve)
        # Keep one key alive: cutting the last key causes Maya to delete the curve.
        cmds.cutKey(curve,clear=True,time=(times[0]+.00001,times[-1]+.00001))
        for i,t in enumerate(times):cmds.setKeyframe(curve,time=t,value=rows[i][k])
        if not (attr.startswith('rotate') and cmds.getAttr(curve+'.rotationInterpolation')>=3):
            cmds.keyTangent(curve,e=True,inTangentType='auto',outTangentType='auto',weightedTangents=False)
        curves[attr]=curve
    _quaternion_keys([curves[a] for a in ATTRS[3:]])
    return curves

def sample_controls(controls,start,end):
    cache={r:dict(start=start,positions=[],rotations=[]) for r in controls}
    for time in range(start,end+1):
        cmds.currentTime(time)
        for role,node in controls.items():
            m=om.MTransformationMatrix(om.MMatrix(cmds.getAttr(node+'.worldMatrix[0]')))
            cache[role]['positions'].append(list(m.translation(om.MSpace.kWorld)))
            cache[role]['rotations'].append(list(m.rotation(asQuaternion=True)))
    return cache

def _roles_from_profile(skeleton,profile):
    mapping={g['goal_name']:g['bone_dag'] for g in profile['ik_goals']}
    roles={role:mapping[goal] for role,_,goal in ROLES[:4] if goal in mapping}
    missing=[role for role,_,_ in ROLES[:4] if role not in roles]
    if missing:raise RuntimeError('未能识别双手双脚，请检查人体骨架命名：'+', '.join(missing))
    roles['COM']=profile['pelvis_bone_dag']
    for role,chain in [('Chest','Spine'),('Head','Head')]:
        if chain in profile['bone_chains']:roles[role]=profile['bone_chains'][chain].end_bone
    return roles

def set_stability(root,amount):
    """Keep spine corrections near input animation without temporal evaluation state."""
    s=state(root);node=resolve(s['solver']);all_joints=joints(root)
    if not cmds.attributeQuery('awChestStability',node=root,exists=True):
        _vec_attr(root,'awChestStability','double',.85,minValue=0,maxValue=.98,keyable=True)
    cmds.setAttr(root+'.awChestStability',max(0.,min(.98,float(amount))))
    existing=s.get('stability_nodes',[])
    if existing:return
    chest_bone=s['role_indices'].get('Chest')
    pelvis=s['role_indices']['COM'];indices=[]
    if chest_bone is not None:
        i=chest_bone
        while i>=0 and i!=pelvis:
            indices.append((i,True));i=s['parents'][i]
    for role in ['L_Hand','R_Hand']:
        i=s['role_indices'][role];ancestor=s['parents'][i]
        while ancestor>=0 and ancestor!=chest_bone and ancestor!=pelvis:
            i=ancestor;ancestor=s['parents'][i]
        if i not in [v[0] for v in indices]:indices.append((i,False))
    created=[]
    for index,spine in indices:
        for attr,factor in [('boneRotationStiffness',1. if spine else .6),('bonePositionStiffness',.25 if spine else 0.)]:
            plug=node+'.'+attr+'[{}]'.format(index)
            if cmds.listConnections(plug,s=True,d=False):continue
            mult=cmds.createNode('multDoubleLinear',name='AW_TorsoStability',skipSelect=True);own(root,mult)
            cmds.connectAttr(root+'.awChestStability',mult+'.input1');cmds.setAttr(mult+'.input2',factor);cmds.connectAttr(mult+'.output',plug);created.append(uid(mult))
    s['stability_nodes']=created;store(root,s)


def set_leg_stability(root,amount=1.):
    """Keep animated knee planes while preserving PBIK hip and foot solutions."""
    import importlib
    from ..core import ue58_pbik_node
    from ..core.pole_vector import build_limb_plane
    s=state(root);node=resolve(s['solver']);cfg=json.loads(cmds.getAttr(node+'.configuration'))
    names=[j.split('|')[-1].split(':')[-1] for j in joints(root)];references=[]
    for role in ['L_Foot','R_Foot']:
        end=s['role_indices'].get(role)
        if end is None:continue
        chain=[];i=cfg['parents'][end]
        while i>=0 and i!=s['role_indices']['COM']:
            if not any(t in names[i].lower() for t in ['twist','roll','helper','proc','correct']):chain.append(i)
            i=cfg['parents'][i]
        if len(chain)<2:continue
        reference=build_limb_plane(cfg['parents'],names,chain[1],chain[0],end,cfg['native_reference'])
        if reference:references.append(reference)
    # Compute-only hot reload: no node schema changes or node deletion needed.
    module=importlib.reload(ue58_pbik_node);selection=om.MSelectionList();selection.add(node)
    instance=om.MFnDependencyNode(selection.getDependNode(0)).userNode()
    type(instance).compute=module.UE58PBIKNode.compute
    amount=max(0.,min(1.,float(amount)))
    if not cmds.attributeQuery('awLegPlaneStability',node=root,exists=True):
        _vec_attr(root,'awLegPlaneStability','double',1.,minValue=0,maxValue=1,keyable=False)
    cmds.setAttr(root+'.awLegPlaneStability',amount)
    cfg['leg_planes']=references;cfg['leg_plane_strength']=amount
    cmds.setAttr(node+'.configuration',json.dumps(cfg),type='string')
    s['leg_plane_version']=1;store(root,s)
    return len(references)

def create(skeleton_root,start,end,reference_frame=0,on_layer=False):
    from ..core.skeleton import RetargetSkeleton
    from ..core.chains import AutoCharacterizer
    from ..core.pbik_native import library,to_engine_pose
    from ..core.math_utils import matrix_to_trs
    start,end=int(start),int(end);reference_frame=int(reference_frame)
    if start>=end:raise ValueError('动画结束帧必须大于开始帧')
    with operation('Create reusable animated full body IK'):
        cmds.namespace(set=':');cmds.currentTime(reference_frame)
        skel=RetargetSkeleton(skeleton_root);profile=AutoCharacterizer.characterize(skel);role_bones=_roles_from_profile(skel,profile)
        original=list(skel.joint_names);offsets=[]
        for bone in original:
            if cmds.listConnections(bone+'.offsetParentMatrix',s=True,d=False) or cmds.getAttr(bone+'.offsetParentMatrix',lock=True):raise RuntimeError('骨架 offsetParentMatrix 已被占用：'+bone)
            offsets.append(list(cmds.getAttr(bone+'.offsetParentMatrix')))
        library()
        plugin=Path(__file__).resolve().parents[1]/'core/ue58_pbik_node.py'
        if 'ue58PBIKSolver' not in cmds.allNodeTypes():cmds.loadPlugin(str(plugin),quiet=True)
        token=uuid.uuid4().hex[:8];root=cmds.createNode('transform',name='AW_FullBodyIK_'+token,skipSelect=True)
        hidden=cmds.createNode('transform',name='AW_SourceAnimation',parent=root,skipSelect=True);cmds.setAttr(hidden+'.visibility',False)
        control_group=cmds.createNode('transform',name='AW_WorldControls',parent=root,skipSelect=True)
        external=cmds.listRelatives(skel.root_joint,p=True,f=True) or []
        if external:cmds.connectAttr(external[0]+'.worldMatrix[0]',hidden+'.offsetParentMatrix')
        namespace='AWSource_'+token;cmds.namespace(add=namespace)
        sources=[]
        for i,bone in enumerate(original):
            clone=cmds.createNode('joint',name=':'+namespace+':'+bone.split('|')[-1].split(':')[-1],parent=sources[skel.parent_indices[i]] if skel.parent_indices[i]>=0 else hidden,skipSelect=True)
            for attr in ['translate','rotate','scale','jointOrient','rotateAxis','inverseScale','rotateOrder','segmentScaleCompensate']:
                cmds.connectAttr(bone+'.'+attr,clone+'.'+attr,force=True)
            cmds.setAttr(clone+'.offsetParentMatrix',*offsets[i],type='matrix');sources.append(cmds.ls(clone,long=True)[0])
        controls={};role_indices={};goals=[]
        radius=max(3.,abs(skel.rest_world_pos[role_bones['COM']].y)*.055)
        for role,_,_ in ROLES:
            if role not in role_bones:continue
            bone=role_bones[role];i=skel.joint_indices[bone]
            c=cmds.circle(name='AW_'+role+'_CTRL',nr=(0,1,0),r=radius*(2. if role=='COM' else 1.),ch=False)[0]
            c=cmds.parent(c,control_group,relative=True)[0];controls[role]=c;role_indices[role]=i
            string(c,'awRole',role);own(root,c)
            color=(.2,.65,1.) if role.startswith('L_') else (1.,.25,.2) if role.startswith('R_') else (1.,.8,.1)
            for shape in cmds.listRelatives(c,s=True,f=True) or []:
                cmds.setAttr(shape+'.overrideEnabled',True);cmds.setAttr(shape+'.overrideRGBColors',True);cmds.setAttr(shape+'.overrideColorRGB',*color);cmds.setAttr(shape+'.lineWidth',2.)
            for attr in ['sx','sy','sz','visibility']:cmds.setAttr(c+'.'+attr,lock=True,keyable=False,channelBox=False)
            for attr,default in [('positionAlpha',1.),('rotationAlpha',1.),('strengthAlpha',.35 if role=='Chest' else .25 if role=='Head' else 1.)]:_vec_attr(c,attr,'double',default,minValue=0,maxValue=1,keyable=True)
            p,q,_=matrix_to_trs(om.MMatrix(cmds.getAttr(bone+'.worldMatrix[0]')))
            goals.append(dict(name=role,bone=i,initial_position=list(p),initial_rotation=list(q)))
        cache=sample_controls({r:role_bones[r] for r in controls},min(start,reference_frame),max(end,reference_frame))
        source_layer=ensure_layer(root,'AW_SourceMotion_'+token,list(controls.values())) if on_layer else None
        for role,c in controls.items():
            data=cache[role];write_pose(root,c,list(range(data['start'],data['start']+len(data['positions']))),data['positions'],data['rotations'],source_layer)
        cmds.currentTime(reference_frame)
        solver=cmds.createNode('ue58PBIKSolver',name='AW_PBIK_'+token,skipSelect=True);own(root,solver);cmds.setAttr(solver+'.enabled',False)
        reference=[]
        for bone in original:
            p,q,scale=matrix_to_trs(om.MMatrix(cmds.getAttr(bone+'.worldMatrix[0]')));reference.append(to_engine_pose(list(p),list(q),list(scale)))
        configuration=dict(parents=skel.parent_indices,root=role_indices['COM'],goals=goals,native_reference=reference,baseline_offsets=offsets,alignment_rotations=[None]*len(original))
        cmds.setAttr(solver+'.configuration',json.dumps(configuration),type='string')
        for i,bone in enumerate(original):cmds.connectAttr(sources[i]+'.matrix',solver+'.inputLocalMatrix[{}]'.format(i))
        if external:cmds.connectAttr(external[0]+'.worldMatrix[0]',solver+'.externalParentMatrix')
        cmds.connectAttr('time1.outTime',solver+'.inputTime')
        for i,goal in enumerate(goals):
            c=controls[goal['name']];cmds.connectAttr(c+'.worldMatrix[0]',solver+'.goalMatrix[{}]'.format(i))
            for attr in ['positionAlpha','rotationAlpha','strengthAlpha']:cmds.connectAttr(c+'.'+attr,solver+'.'+attr+'[{}]'.format(i))
            for attr,v in [('previewMode',1),('pullChainAlpha',0.),('chainDepth',0),('pinRotation',1.)]:cmds.setAttr(solver+'.'+attr+'[{}]'.format(i),v)
        _vec_attr(root,'enabled','bool',True,keyable=True);_vec_attr(root,'iterations','long',200,minValue=1,maxValue=1000,keyable=True)
        cmds.connectAttr(root+'.enabled',solver+'.enabled');cmds.connectAttr(root+'.iterations',solver+'.iterations')
        for i,bone in enumerate(original):cmds.connectAttr(solver+'.outputOffset[{}]'.format(i),bone+'.offsetParentMatrix')
        s=dict(schema=SCHEMA,id=token,root_joint=uid(original[0]),joints=[uid(j) for j in original],sources=[uid(j) for j in sources],parents=skel.parent_indices,offsets=offsets,
            solver=uid(solver),controls={r:uid(c) for r,c in controls.items()},role_indices=role_indices,start=start,end=end,reference_frame=reference_frame,
            source_layer=source_layer,source_on_layer=bool(on_layer),source_namespace=namespace,control_group=uid(control_group),rhythm_layers={},settings=dict(engine.DEFAULTS))
        store(root,s);string(root,'awSourceCache',_pack(cache))
        for n in cmds.listRelatives(root,ad=True,f=True) or []:own(root,n)
        set_stability(root,.85)
        set_leg_stability(root,1.)
        return root

def adopt_legacy(root):
    if cmds.attributeQuery('awState',node=root,exists=True):return root
    legacy=json.loads(cmds.getAttr(root+'.rigManifest'));solver=legacy['solver'];cfg=json.loads(cmds.getAttr(solver+'.configuration'))
    with operation('Adopt existing animated full body IK'):
        controls={r:legacy['controls'][r+'_IK'] for r in ['L_Foot','R_Foot','L_Hand','R_Hand']}
        for role,name in [('COM','Pelvis_IK'),('Chest','Chest_IK'),('Head','Head_IK')]:
            if name in legacy['controls']:controls[role]=legacy['controls'][name]
        layers={r:r+'_Rhythm_Edit' for r in controls if cmds.objExists(r+'_Rhythm_Edit')}
        weights={l:cmds.getAttr(l+'.weight') for l in layers.values()}
        start=int(cmds.playbackOptions(q=True,min=True));end=int(cmds.playbackOptions(q=True,max=True));ref=0
        try:
            for l in layers.values():cmds.setAttr(l+'.weight',0.)
            cache=sample_controls(controls,min(start,ref),max(end,ref))
        finally:
            for l,w in weights.items():cmds.setAttr(l+'.weight',w)
        s=dict(schema=SCHEMA,id=uuid.uuid4().hex[:8],root_joint=uid(legacy['joints'][0]),joints=[uid(j) for j in legacy['joints']],sources=[uid(j) for j in legacy['sources']],
            parents=cfg['parents'],offsets=cfg['baseline_offsets'],solver=uid(solver),controls={r:uid(c) for r,c in controls.items()},
            role_indices={},start=start,end=end,reference_frame=ref,source_layer=None,source_on_layer=False,
            source_namespace=legacy['sources'][0].split('|')[-1].rsplit(':',1)[0],control_group=uid('FullBodyIK_Controls'),rhythm_layers=layers,settings=dict(engine.DEFAULTS),adopted=True)
        for role,c in controls.items():
            index=next((i for i,g in enumerate(cfg['goals']) if (cmds.listConnections(solver+'.goalMatrix[{}]'.format(i),s=True,d=False) or [None])[0]==c),None)
            if index is None:raise RuntimeError('原有控制器与求解器的连接不匹配：'+c)
            s['role_indices'][role]=cfg['goals'][index]['bone']
        store(root,s);string(root,'awSourceCache',_pack(cache));own(root,solver)
        group=resolve(s['control_group'])
        for role,c in controls.items():
            string(c,'awRole',role);own(root,c)
            if role in ['Chest','Head']:cmds.parent(c,group,absolute=True)
            data=cache[role];write_pose(root,c,list(range(data['start'],data['start']+len(data['positions']))),data['positions'],data['rotations'])
        for l in layers.values():
            own(root,l);cmds.setAttr(l+'.rotationAccumulationMode',1)
            for curve in cmds.animLayer(l,q=True,animCurves=True) or []:own(root,curve)
        for n in cmds.listRelatives(root,ad=True,f=True) or []:own(root,n)
        set_stability(root,.85)
        set_leg_stability(root,1.)
        return root

def apply_rhythm(root,roles,settings=None,marker_nodes=None):
    from . import markers
    from . import rhythm_layers
    with operation('Apply bookmark-driven integer-frame rhythm'):
        s=state(root);cache=source_cache(root)
        if settings:
            selected=marker_nodes if marker_nodes else [m['node'] for m in markers.list_markers(root,roles)]
            for node in selected:markers.settings(node,settings)
            s['settings']=dict(settings);store(root,s)
        result={}
        for role in roles:
            if role not in cache:continue
            points=markers.list_markers(root,[role])
            result[role]=rhythm_layers.build(root,s,role,cache[role],points)
        store(root,s)
        return result

def restore_controls(root,roles):
    with operation('Restore controller source motion'):
        s=state(root)
        for role in roles:
            layer=s['rhythm_layers'].get(role)
            if layer and cmds.objExists(layer):cmds.setAttr(layer+'.weight',0.)

def recapture_controls(root):
    with operation('Capture controller reference animation'):
        s=state(root);weights={l:cmds.getAttr(l+'.weight') for l in s['rhythm_layers'].values() if cmds.objExists(l)}
        try:
            for layer in weights:cmds.setAttr(layer+'.weight',0.)
            cache=sample_controls({r:controller(root,r) for r in s['controls']},min(s['start'],s['reference_frame']),max(s['end'],s['reference_frame']))
        finally:
            for layer,weight in weights.items():cmds.setAttr(layer+'.weight',weight)
        string(root,'awSourceCache',_pack(cache))

def _restore_offsets(root):
    s=state(root)
    solver=resolve(s['solver'])
    for joint in joints(root):
        inputs=cmds.listConnections(joint+'.offsetParentMatrix',s=True,d=False) or []
        if any(uid(node)!=uid(solver) for node in inputs):raise RuntimeError('骨骼存在其他工具的新连接，请先处理：'+joint+'.offsetParentMatrix')
    for joint,offset in zip(joints(root),s['offsets']):
        for plug in cmds.listConnections(joint+'.offsetParentMatrix',s=True,d=False,p=True) or []:cmds.disconnectAttr(plug,joint+'.offsetParentMatrix')
        cmds.setAttr(joint+'.offsetParentMatrix',*offset,type='matrix')

def _delete_nodes(root):
    s=state(root);all_owned=owned(root)
    # The original skeleton is never an owned node.
    originals=set(s['joints'])
    if any(uid(n) in originals for n in all_owned):raise RuntimeError('所有权检查失败：原始骨架出现在删除列表中')
    outside=[n for n in all_owned if not cmds.objectType(n,isAType='dagNode')]
    if outside:cmds.delete(outside)
    if cmds.objExists(root):cmds.delete(root)
    ns=s.get('source_namespace')
    if ns and cmds.namespace(exists=ns) and not (cmds.namespaceInfo(ns,listOnlyDependencyNodes=True) or []):cmds.namespace(removeNamespace=ns)

def remove_restore(root):
    with operation('Remove IK and preserve source animation'):
        _restore_offsets(root);_delete_nodes(root)

def remove_bake(root):
    """Bake evaluated bone channels to an independent result layer, then remove IK."""
    with operation('Bake skeleton and remove animated IK'):
        s=state(root);bones=joints(root);times=list(range(min(s['start'],s['reference_frame']),max(s['end'],s['reference_frame'])+1));world=[]
        for t in times:
            cmds.currentTime(t);world.append([list(cmds.getAttr(j+'.worldMatrix[0]')) for j in bones])
        parent=cmds.listRelatives(bones[0],p=True,f=True) or []
        temp=cmds.createNode('transform',name='AW_BakeConversion_TEMP',skipSelect=True)
        if parent:cmds.connectAttr(parent[0]+'.worldMatrix[0]',temp+'.offsetParentMatrix')
        clones=[];channels={i:[] for i in range(len(bones))}
        try:
            for i,j in enumerate(bones):
                c=cmds.createNode('joint',name='AW_BakeJoint_TEMP',parent=clones[s['parents'][i]] if s['parents'][i]>=0 else temp,skipSelect=True)
                c=cmds.ls(c,long=True)[0] if '|' in c else (clones[s['parents'][i]] if s['parents'][i]>=0 else '|'+temp)+'|'+c
                for a in ['jointOrient','rotateAxis','translate','rotate','scale']:cmds.setAttr(c+'.'+a,*cmds.getAttr(j+'.'+a)[0])
                for a in ['rotateOrder','segmentScaleCompensate']:cmds.setAttr(c+'.'+a,cmds.getAttr(j+'.'+a))
                parent_index=s['parents'][i]
                if parent_index>=0:cmds.connectAttr(clones[parent_index]+'.scale',c+'.inverseScale',force=True)
                else:cmds.connectAttr(j+'.inverseScale',c+'.inverseScale',force=True)
                cmds.setAttr(c+'.offsetParentMatrix',*s['offsets'][i],type='matrix');clones.append(c)
            for f,t in enumerate(times):
                cmds.currentTime(t)
                for i,c in enumerate(clones):
                    cmds.xform(c,ws=True,matrix=world[f][i]);channels[i].append([list(cmds.getAttr(c+'.'+a)[0]) for a in ['translate','rotate','scale']])
            _restore_offsets(root)
            layer=cmds.animLayer('AW_BakedSkeleton',override=True);cmds.setAttr(layer+'.rotationAccumulationMode',1)
            bone_attrs=ATTRS+['scaleX','scaleY','scaleZ'];cmds.animLayer(layer,e=True,attribute=[j+'.'+a for j in bones for a in bone_attrs])
            for i,j in enumerate(bones):
                rotation_curves=[]
                for k,a in enumerate(bone_attrs):
                    group=k//3;component=k%3;cmds.setKeyframe(j,at=a,t=times[0],v=channels[i][0][group][component],animLayer=layer)
                    curve=layer_curve(layer,j,a)
                    for f,t in enumerate(times):cmds.setKeyframe(curve,t=t,v=channels[i][f][group][component])
                    if 3<=k<6:rotation_curves.append(curve)
                _quaternion_keys(rotation_curves)
            # Validate the conversion before deleting any rig-owned nodes.
            maximum=0.
            for f in sorted(set([0,len(times)//2,len(times)-1])):
                cmds.currentTime(times[f])
                for i,j in enumerate(bones):
                    actual=cmds.getAttr(j+'.worldMatrix[0]');maximum=max(maximum,max(abs(a-b) for a,b in zip(actual,world[f][i])))
            if maximum>1e-3:raise RuntimeError('烘焙验证失败，已回滚。矩阵误差：'+str(maximum))
            _delete_nodes(root)
            return layer
        finally:
            if cmds.objExists(temp):cmds.delete(temp)
