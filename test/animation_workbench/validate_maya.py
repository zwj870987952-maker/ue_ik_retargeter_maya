"""Run with Maya 2025 mayapy. Creates and modifies only standalone fixtures."""
from pathlib import Path
import json,math,sys,tempfile,traceback
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ue58_three_assets_v2'))
import maya.standalone
maya.standalone.initialize(name='python')
import maya.cmds as cmds
import maya.api.OpenMaya as om
from ue_ik_retargeter_maya.animation_workbench import scene,markers,engine,rhythm_layers
from ue_ik_retargeter_maya.core import pole_vector as pole

def fixture():
    cmds.file(new=True,force=True)
    group=cmds.createNode('transform',name='Character');cmds.setAttr(group+'.translate',4,2,-3);cmds.setAttr(group+'.rotateY',17);cmds.setAttr(group+'.scale',1.1,1.1,1.1)
    bones={}
    def j(name,parent,p):
        node=cmds.createNode('joint',name=name,parent=bones.get(parent,group));cmds.setAttr(node+'.translate',*p);bones[name]=cmds.ls(node,long=True)[0];return node
    j('root',None,(0,0,0));j('pelvis','root',(0,95,0));j('spine_01','pelvis',(0,13,0));j('spine_02','spine_01',(0,13,0));j('spine_03','spine_02',(0,13,0));j('neck_01','spine_03',(0,12,0));j('head','neck_01',(0,10,0))
    for side,sign in [('l',1),('r',-1)]:
        j('clavicle_'+side,'spine_03',(sign*8,6,0));j('upperarm_'+side,'clavicle_'+side,(sign*12,0,0));j('lowerarm_'+side,'upperarm_'+side,(sign*25,-2,1));j('hand_'+side,'lowerarm_'+side,(sign*23,0,0))
        j('thigh_'+side,'pelvis',(sign*9,-4,0));j('calf_'+side,'thigh_'+side,(0,-42,2));j('foot_'+side,'calf_'+side,(0,-42,-1));j('ball_'+side,'foot_'+side,(0,-4,10))
    for name,node in bones.items():
        if name!='root':cmds.setAttr(node+'.jointOrient',2,3,-1)
    for t in range(13):
        for name,axis,amp in [('pelvis','translateX',3),('spine_02','rotateY',6),('upperarm_l','rotateZ',20),('upperarm_r','rotateZ',-20),('calf_l','rotateX',8),('calf_r','rotateX',-8)]:
            cmds.setKeyframe(bones[name],at=axis,t=t,v=0 if t==0 else math.sin(t*.5)*amp)
    cmds.playbackOptions(min=1,max=12);cmds.currentTime(7);cmds.select(bones['pelvis'])
    unrelated_layer=cmds.animLayer('UnrelatedLayer',override=True);cmds.animLayer(unrelated_layer,e=True,attribute=group+'.visibility')
    return bones

def matrices(bones):
    result=[]
    for t in range(13):cmds.currentTime(t);result.append([list(cmds.getAttr(j+'.worldMatrix[0]')) for j in bones.values()])
    return result

def difference(a,b):return max(abs(x-y) for f,g in zip(a,b) for m,n in zip(f,g) for x,y in zip(m,n))

def curve_snapshot():
    return {c:(cmds.keyframe(c,q=True,tc=True),cmds.keyframe(c,q=True,vc=True)) for c in cmds.ls(type='animCurve') or []}

def validate_knee_planes(root):
    s=scene.state(root);node=scene.resolve(s['solver']);cfg=json.loads(cmds.getAttr(node+'.configuration'));bones=scene.joints(root)
    assert len(cfg['leg_planes'])==2
    initial=curve_snapshot();maximum=0.
    for role,delta in [('L_Hand',(20,15,0)),('R_Foot',(10,12,0)),('COM',(0,-10,0))]:
        goal=next(i for i,g in enumerate(cfg['goals']) if g['name']==role);plug=node+'.goalMatrix[{}]'.format(goal);source=cmds.listConnections(plug,s=True,d=False,p=True)[0]
        m=list(cmds.getAttr(plug));m[12:15]=[a+b for a,b in zip(m[12:15],delta)]
        with scene.operation('Knee plane integration fixture'):
            try:
                cmds.disconnectAttr(source,plug);cmds.setAttr(plug,*m,type='matrix');scene.set_leg_stability(root,0.)
                before=[list(cmds.getAttr(j+'.worldMatrix[0]')) for j in bones];scene.set_leg_stability(root,1.)
                after=[list(cmds.getAttr(j+'.worldMatrix[0]')) for j in bones]
                for ref in cfg['leg_planes']:
                    hip,knee,end=[ref[k] for k in ('hip','knee','end')]
                    maximum=max(maximum,max(abs(a-b) for a,b in zip(before[end],after[end])),pole.length(pole.sub(before[hip][12:15],after[hip][12:15])))
                    inputs=[cmds.getAttr(scene.resolve(s['sources'][i])+'.worldMatrix[0]')[12:15] for i in [hip,knee,end]]
                    original=pole.bend_measure(*inputs);output=pole.bend_measure(*[after[i][12:15] for i in [hip,knee,end]])
                    if original['height_ratio']>1e-5 and output['height_ratio']>1e-8:
                        desired=pole.transport_direction(original['axis'],pole.unit(original['pole']),output['axis'])
                        assert pole.dot(desired,pole.unit(output['pole']))>.99999
            finally:cmds.connectAttr(source,plug,force=True);scene.set_leg_stability(root,1.)
    assert maximum<1e-5,maximum
    assert curve_snapshot()==initial
    return maximum

def validate_live_weights(root,role):
    s=scene.state(root);info=s['rhythm_components'][role];c=scene.controller(root,role);cache=scene.source_cache(root)[role]
    points=markers.list_markers(root,[role]);models=engine.layer_components(cache,points,s['start'],s['end']);pose_curves=[]
    for identifier in info['layers'].values():pose_curves.extend(cmds.animLayer(scene.resolve(identifier),q=True,animCurves=True) or [])
    snapshot={curve:(cmds.keyframe(curve,q=True,tc=True),cmds.keyframe(curve,q=True,vc=True)) for curve in pose_curves};layer=s['rhythm_layers'][role]
    maximum=0.
    for gains in [(0,0,0),(100,0,0),(0,100,0),(0,0,100),(40,25,75),(100,100,100)]:
        with rhythm_layers.edit_weights():
            for part,value in zip(rhythm_layers.PARTS,gains):rhythm_layers.set_strength(root,[role],part,value)
        for overall in [0.,.35,1.]:
            cmds.setAttr(layer+'.weight',overall)
            for t in range(13):
                cmds.currentTime(t);m=om.MTransformationMatrix(om.MMatrix(cmds.getAttr(c+'.worldMatrix[0]')))
                expected=engine.lerp(cache['positions'][t],models['hold']['positions'][t],overall*gains[0]/100.)
                expected=engine.add(expected,engine.mul(models['settle'][t],overall*gains[1]/100.));expected=engine.add(expected,engine.mul(models['prep'][t],overall*gains[2]/100.))
                maximum=max(maximum,engine.length(engine.sub(list(m.translation(om.MSpace.kWorld)),expected)))
                angle=engine.quaternion_angle(list(m.rotation(asQuaternion=True)),engine.slerp(cache['rotations'][t],models['hold']['rotations'][t],overall*gains[0]/100.))
                gain_curve=scene.resolve(info['weight_curves']['hold'])
                assert angle<.001,(gains,overall,t,angle,cmds.getAttr(scene.resolve(info['layers']['hold'])+'.weight'),cmds.keyframe(gain_curve,q=True,tc=True),cmds.keyframe(gain_curve,q=True,vc=True),cmds.keyTangent(gain_curve,q=True,ott=True),info['events'])
    # A selected bookmark changes only its sparse gain key, not other events.
    if points:
        target=points[0]['node'];rhythm_layers.set_strength(root,[role],'hold',37,[target]);target_id=scene.uid(target)
        curve=scene.resolve(info['weight_curves']['hold'])
        for event in info['events']:
            expected=.37 if event['marker']==target_id else 1.
            assert abs(cmds.keyframe(curve,q=True,time=(event['a'],event['a']),vc=True)[0]-expected)<1e-9
        rhythm_layers.set_strength(root,[role],'hold',100)
    state_before=cmds.getAttr(root+'.awState');auto=cmds.autoKeyframe(q=True,state=True)
    with rhythm_layers.edit_weights():rhythm_layers.set_strength(root,[role],'hold',19)
    assert cmds.undoInfo(q=True,undoName=True)=='Live rhythm layer weights'
    cmds.undo();assert cmds.getAttr(root+'.awState')==state_before and cmds.autoKeyframe(q=True,state=True)==auto
    assert maximum<1e-5,maximum
    assert all((cmds.keyframe(curve,q=True,tc=True),cmds.keyframe(curve,q=True,vc=True))==v for curve,v in snapshot.items())
    return maximum

results={}
try:
    for on_layer in [False,True]:
        bones=fixture();baseline=matrices(bones);original=curve_snapshot();cmds.currentTime(7);selection=cmds.ls(sl=True)
        root=scene.create(bones['root'],1,12,0,on_layer);assert cmds.currentTime(q=True)==7 and cmds.ls(sl=True)==selection
        assert all(curve_snapshot()[c]==v for c,v in original.items())
        delta=difference(baseline,matrices(bones));assert delta<.001,('Initial pose differs',delta)
        s=scene.state(root);assert bool(s['source_layer'])==on_layer
        results['knee_plane_endpoint_error']=validate_knee_planes(root)
        c=scene.controller(root,'R_Hand');cache=scene.source_cache(root)
        unrelated=markers.add(root,['L_Hand'],5)[0];cmds.disconnectAttr(root+'.message',unrelated+'.awOwner');cmds.disconnectAttr(unrelated+'.message',(cmds.listConnections(unrelated+'.message',s=False,d=True,p=True) or [])[0])
        before=curve_snapshot();markers.generate(root,scene.DEFAULT_ROLES,2,3);assert curve_snapshot()==before
        nodes=markers.add(root,['R_Hand'],6);values=dict(engine.DEFAULTS,hold_left=1,hold_right=2,support_frames=5,settle_duration=80,prep_duration=80)
        assert all(not m['automatic'] for m in markers.list_markers(root,['R_Hand']) if m['frame']==6)
        scene.apply_rhythm(root,['R_Hand'],values,nodes)
        # Regeneration must not reset X/Y of the first quaternion key while
        # creating/keying Z through a By-Layer animation blend.
        scene.apply_rhythm(root,['R_Hand'],values,nodes)
        results['live_weight_position_error']=validate_live_weights(root,'R_Hand')
        layer=scene.state(root)['rhythm_layers']['R_Hand'];assert cmds.getAttr(layer+'.rotationAccumulationMode')==1
        model=engine.evaluate(cache['R_Hand'],markers.list_markers(root,['R_Hand']),1,12)
        maximum=0.
        for weight in [0.,.25,.5,.6,.75,1.]:
            cmds.setAttr(layer+'.weight',weight)
            for t in range(13):
                cmds.currentTime(t);q=list(om.MTransformationMatrix(om.MMatrix(cmds.getAttr(c+'.worldMatrix[0]'))).rotation(asQuaternion=True))
                expected=engine.slerp(cache['R_Hand']['rotations'][t],model['rotations'][t],weight);maximum=max(maximum,engine.quaternion_angle(q,expected))
        assert maximum<.001,maximum
        parts=scene.state(root)['rhythm_components']['R_Hand']
        for identifier in parts['layers'].values():
            for curve in cmds.animLayer(scene.resolve(identifier),q=True,animCurves=True) or []:assert all(t==int(t) for t in cmds.keyframe(curve,q=True,tc=True))
        markers.delete(root,nodes);assert not any(m['frame']==6 for m in markers.list_markers(root,['R_Hand']))
        scene.apply_rhythm(root,['R_Hand']);assert cmds.objExists(unrelated)
        # Persisted UUIDs, source cache and native node survive save/reopen.
        filename=str(Path(tempfile.gettempdir())/'animation_workbench_fixture.ma');cmds.file(rename=filename);cmds.file(save=True,type='mayaAscii',force=True);cmds.file(filename,open=True,force=True)
        root=scene.roots()[0];assert len(scene.source_cache(root)['R_Hand']['positions'])==13
        if on_layer:
            cmds.setAttr(scene.state(root)['rhythm_layers']['R_Hand']+'.weight',1.)
            before=matrices(bones);baked=scene.remove_bake(root);after=matrices(bones);error=difference(before,after);assert error<.001,error
            assert cmds.objExists(baked) and not scene.roots();cmds.setAttr(baked+'.weight',0.);assert difference(baseline,matrices(bones))<.001
            results['bake_error']=error
        else:
            scene.remove_restore(root);assert difference(baseline,matrices(bones))<.001;assert all(curve_snapshot()[c]==v for c,v in original.items());assert not scene.roots()
        assert cmds.objExists(unrelated)
        results['source_layer' if on_layer else 'direct']=dict(initial_error=delta,rotation_error=maximum,original_curves=len(original),passed=True)
    cmds.undoInfo(stateWithoutFlush=False)
    try:
        with scene.operation('Transaction rollback fixture'):
            cmds.createNode('transform',name='AW_FailedOperation_TEMP')
            raise ValueError('Deliberate rollback test')
    except ValueError:pass
    assert not cmds.objExists('AW_FailedOperation_TEMP') and not cmds.undoInfo(q=True,state=True)
    cmds.undoInfo(stateWithoutFlush=True)
    results['rollback_with_undo_disabled']=True
    print('WORKBENCH_VALIDATION '+json.dumps(results))
except Exception:
    traceback.print_exc();sys.exit(1)
finally:
    maya.standalone.uninitialize()
