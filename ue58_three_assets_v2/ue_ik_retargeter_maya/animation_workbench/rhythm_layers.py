"""Native animation-layer targets with cheap, independent live weight curves."""
from contextlib import contextmanager
import maya.cmds as cmds
from . import scene,engine,markers

PARTS=('hold','settle','prep')

@contextmanager
def edit_weights():
    """One undo per drag, without moving the playhead during playback."""
    undo=cmds.undoInfo(q=True,state=True);auto=cmds.autoKeyframe(q=True,state=True)
    if not undo:cmds.undoInfo(stateWithoutFlush=True)
    cmds.undoInfo(openChunk=True,chunkName='Live rhythm layer weights');cmds.autoKeyframe(state=False);failed=False
    try:yield
    except Exception:failed=True;raise
    finally:
        cmds.autoKeyframe(state=auto);cmds.undoInfo(closeChunk=True)
        if failed:cmds.undo()
        if not undo:cmds.undoInfo(stateWithoutFlush=False)

def _connect(source,target):
    if not cmds.isConnected(source,target):cmds.connectAttr(source,target,force=True)

def _translation_keys(root,layer,control,times,values):
    attrs=scene.ATTRS[:3];cmds.animLayer(layer,e=True,attribute=[control+'.'+a for a in attrs])
    missing=[a for a in attrs if not scene.layer_curve(layer,control,a)]
    if missing:cmds.setKeyframe(control,at=missing,t=times[0],animLayer=layer)
    for k,a in enumerate(attrs):
        curve=scene.layer_curve(layer,control,a);scene.own(root,curve)
        cmds.cutKey(curve,clear=True,time=(times[0]+.00001,times[-1]+.00001))
        for t,row in zip(times,values):cmds.setKeyframe(curve,t=t,v=row[k])
        cmds.keyTangent(curve,e=True,itt='auto',ott='auto',weightedTangents=False)

def _weight_curve(root,name,events,points,part,start,end,existing=None):
    curve=scene.resolve(existing) if existing and cmds.ls(existing) else cmds.createNode('animCurveTU',name=name,skipSelect=True)
    scene.own(root,curve)
    if not cmds.listConnections(curve+'.input',s=True,d=False):cmds.connectAttr('time1.outTime',curve+'.input')
    values={start:1.,end+1:1.};settings={scene.uid(m['node']):m['settings'] for m in points}
    for event in events:
        values[event['a']]=settings[event['marker']].get(part+'_strength',100.)/100.
        values[event['c']+1]=1.
    times=sorted(values);cmds.setKeyframe(curve,t=times[0],v=values[times[0]])
    cmds.cutKey(curve,clear=True,time=(times[0]+.00001,times[-1]+.00001))
    for t in times:cmds.setKeyframe(curve,t=t,v=values[t])
    cmds.keyTangent(curve,e=True,weightedTangents=False)
    cmds.keyTangent(curve,e=True,itt='linear',ott='step')
    return curve

def build(root,s,role,source,points):
    control=scene.resolve(s['controls'][role]);components=engine.layer_components(source,points,s['start'],s['end'],role.endswith('Foot'))
    model=components['hold'];main=s['rhythm_layers'].get(role)
    if not main or not cmds.objExists(main):main=cmds.animLayer('AW_Rhythm_'+role+'_'+s['id'],override=True)
    scene.own(root,main);s['rhythm_layers'][role]=main
    cmds.setAttr(main+'.rotationAccumulationMode',1)
    # Keep the existing native layer and its weight as the overall driver.
    # Separate flat component layers avoid Maya group layers masking BaseAnimation.
    attrs=cmds.animLayer(main,q=True,attribute=True) or []
    if attrs:cmds.animLayer(main,e=True,removeAttribute=attrs)
    info=s.setdefault('rhythm_components',{}).get(role,{})
    events=[dict(marker=scene.uid(e['marker']),frame=e['frame'],a=e['a'],c=e['c']) for e in model['events']]
    layers={};weight_curves={};multipliers={}
    for part in PARTS:
        previous=info.get('layers',{}).get(part)
        layer=scene.resolve(previous) if previous and cmds.ls(previous) else cmds.animLayer('AW_'+role+'_'+part.title()+'_'+s['id'],override=part=='hold')
        scene.own(root,layer);cmds.setAttr(layer+'.rotationAccumulationMode',1)
        if part=='hold':
            cmds.animLayer(layer,e=True,attribute=[control+'.'+a for a in scene.ATTRS])
            scene.write_pose(root,control,model['times'],model['positions'],model['rotations'],layer)
        else:_translation_keys(root,layer,control,model['times'],components[part])
        curve=_weight_curve(root,'AW_'+role+'_'+part+'_Gain',events,points,part,source['start'],model['times'][-1],info.get('weight_curves',{}).get(part))
        previous=info.get('multipliers',{}).get(part)
        mult=scene.resolve(previous) if previous and cmds.ls(previous) else cmds.createNode('multDoubleLinear',name='AW_'+role+'_'+part+'_Weight',skipSelect=True)
        scene.own(root,mult);_connect(main+'.weight',mult+'.input1');_connect(curve+'.output',mult+'.input2');_connect(mult+'.output',layer+'.weight')
        layers[part]=scene.uid(layer);weight_curves[part]=scene.uid(curve);multipliers[part]=scene.uid(mult)
    s['rhythm_components'][role]=dict(layers=layers,weight_curves=weight_curves,multipliers=multipliers,events=events)
    return dict(poses=len(points),frames=len(model['times']),layer=main)

def set_strength(root,roles,part,value,marker_nodes=None):
    """Edit sparse native gain keys and bookmark metadata, never pose curves."""
    s=scene.state(root);key=part+'_strength';value=max(0.,min(100.,float(value)))
    selected=set(marker_nodes or []);affected=0
    for role in roles:
        points=markers.list_markers(root,[role]);targets=[m for m in points if not selected or m['node'] in selected]
        target_ids={scene.uid(m['node']) for m in targets}
        for m in targets:
            values=m['settings'];values[key]=value;markers.settings(m['node'],values)
        info=s.get('rhythm_components',{}).get(role)
        if not info:continue
        curve=scene.resolve(info['weight_curves'][part])
        for event in info['events']:
            if event['marker'] in target_ids:
                cmds.keyframe(curve,e=True,time=(event['a'],event['a']),absolute=True,valueChange=value/100.);affected+=1
        cmds.keyTangent(curve,e=True,itt='linear',ott='step')
    s['settings'][key]=value;scene.store(root,s)
    return affected
