"""Owned one-frame native Maya bookmarks are the authoritative key poses."""
import json
import maya.cmds as cmds
from . import engine,scene

LABELS={r:label for r,label,_ in scene.ROLES}
COLORS={'L_Hand':(.25,.65,1.),'R_Hand':(1.,.35,.25),'L_Foot':(.25,.85,.65),
        'R_Foot':(.95,.65,.25),'COM':(.8,.5,1.),'Chest':(.85,.8,.25),'Head':(.5,.8,.9)}

def belongs(root,node):
    if not cmds.objExists(node) or not cmds.attributeQuery('awRole',node=node,exists=True):return False
    owners=cmds.listConnections(node+'.awOwner',s=True,d=False) or [] if cmds.attributeQuery('awOwner',node=node,exists=True) else []
    return bool(owners and scene.uid(owners[0])==scene.uid(root))

def settings(node,value=None):
    if value is not None:scene.string(node,'awSettings',json.dumps(value,ensure_ascii=False))
    result=dict(engine.DEFAULTS)
    if cmds.attributeQuery('awSettings',node=node,exists=True):result.update(json.loads(cmds.getAttr(node+'.awSettings')))
    return result

def list_markers(root,roles=None):
    result=[]
    for node in cmds.ls(type='timeSliderBookmark') or []:
        if not belongs(root,node):continue
        role=cmds.getAttr(node+'.awRole')
        if roles is not None and role not in roles:continue
        frame=int(round(cmds.getAttr(node+'.timeRangeStart')))
        automatic=cmds.getAttr(node+'.awAutomatic') if cmds.attributeQuery('awAutomatic',node=node,exists=True) else False
        detected=cmds.getAttr(node+'.awDetectedFrame') if cmds.attributeQuery('awDetectedFrame',node=node,exists=True) else frame
        result.append(dict(node=node,role=role,frame=frame,settings=settings(node),automatic=bool(automatic and frame==detected)))
    return sorted(result,key=lambda m:(m['role'],m['frame']))

def _tag(root,node,role,frame,automatic,values):
    scene.own(root,node);scene.string(node,'awRole',role);settings(node,values)
    for name,kind,value in [('awAutomatic','bool',automatic),('awDetectedFrame','long',frame)]:
        if not cmds.attributeQuery(name,node=node,exists=True):cmds.addAttr(node,ln=name,at=kind)
        cmds.setAttr(node+'.'+name,value)
    cmds.setAttr(node+'.timeRangeStart',frame);cmds.setAttr(node+'.timeRangeStop',frame+1)
    cmds.setAttr(node+'.name',LABELS.get(role,role)+' · B '+str(frame),type='string')
    cmds.setAttr(node+'.color',*COLORS.get(role,(1.,.7,.2)))
    return node

def add(root,roles,frame=None,automatic=False,values=None):
    from maya.plugin.timeSliderBookmark.timeSliderBookmark import createBookmark
    frame=int(round(cmds.currentTime(q=True))) if frame is None else int(frame)
    s=scene.state(root)
    if not s['start']<=frame<=s['end']:raise ValueError('重点 pose 必须位于动画范围内；参考 pose 不参与节奏处理')
    with scene.operation('Add rhythm pose bookmark'):
        if not cmds.pluginInfo('timeSliderBookmark',q=True,loaded=True):cmds.loadPlugin('timeSliderBookmark',quiet=True)
        existing={(m['role'],m['frame']):m['node'] for m in list_markers(root)};created=[]
        for role in roles:
            if (role,frame) in existing:
                node=existing[(role,frame)]
                if not automatic:cmds.setAttr(node+'.awAutomatic',False)
                created.append(node);continue
            # Autodesk's UI helper requires $gPlayBackSlider even with explicit
            # bounds. A batch fixture has no time slider; use the same native node.
            node=cmds.createNode('timeSliderBookmark') if cmds.about(batch=True) else createBookmark(name='',start=frame,stop=frame+1,color=COLORS.get(role,(1.,.7,.2)))
            _tag(root,node,role,frame,automatic,values or s['settings']);created.append(node)
        return created

def delete(root,nodes):
    targets=[n for n in nodes if belongs(root,n)]
    if not targets:raise ValueError('请先选择要删除的本工具书签')
    with scene.operation('Delete rhythm pose bookmarks'):cmds.delete(targets)
    return len(targets)

def generate(root,roles,min_gap=30,max_count=8):
    s=scene.state(root);cache=scene.source_cache(root);result={}
    with scene.operation('Identify and create pose bookmarks only'):
        for role in roles:
            current=list_markers(root,[role]);manual={m['frame'] for m in current if not m['automatic']}
            detected=engine.detect(cache[role],s['start'],s['end'],min_gap,max_count)
            keep=set(detected)|manual
            obsolete=[m['node'] for m in current if m['automatic'] and m['frame'] not in keep]
            if obsolete:cmds.delete(obsolete)
            for frame in detected:add(root,[role],frame,automatic=True)
            result[role]=detected
    return result

def migrate_legacy(root):
    """Convert only the old example's markers; unrelated bookmarks stay untouched."""
    s=scene.state(root)
    with scene.operation('Migrate rhythm bookmarks'):
        old_hand=[n for n in cmds.ls(type='timeSliderBookmark') or [] if cmds.attributeQuery('leftRhythmOwner',node=n,exists=True) and not belongs(root,n)]
        for node in old_hand:
            b=int(round((cmds.getAttr(node+'.timeRangeStart')+cmds.getAttr(node+'.timeRangeStop'))*.5))
            _tag(root,node,'L_Hand',b,False,s['settings'])
        for role,layer in s['rhythm_layers'].items():
            # Native bookmarks may have been manually moved since the example
            # stored its CSV. Their actual positions must take precedence.
            if role=='L_Hand' and old_hand:continue
            if not cmds.objExists(layer) or not cmds.attributeQuery('poseFrames',node=layer,exists=True):continue
            for token in cmds.getAttr(layer+'.poseFrames').split(','):
                if token.strip():add(root,[role],int(token),automatic=True)
