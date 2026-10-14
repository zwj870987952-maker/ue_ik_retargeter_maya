"""Stable limb planes and preferred-angle calibration, independent of Maya.

Three collinear joint positions cannot determine an anatomical pole direction.
A captured bind plane, foot-to-toe direction or signed preferred angle supplies
that missing information. References are stored in the upper bone's frame, so
evaluation remains deterministic when scrubbing frames in any order.
"""
import math

STRAIGHT_DEGREES = 3.0
RELIABLE_DEGREES = 8.0

def add(a,b):return [x+y for x,y in zip(a,b)]
def sub(a,b):return [x-y for x,y in zip(a,b)]
def mul(a,s):return [x*s for x in a]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def length(a):return math.sqrt(dot(a,a))
def unit(a):
    n=length(a)
    return mul(a,1./n) if n>1e-12 else [0.,0.,0.]
def project(a,axis):return sub(a,mul(axis,dot(a,axis)))
def qmul(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]
def inverse(q):return [-q[0],-q[1],-q[2],q[3]]
def rotate(v,q):
    t=mul(cross(q[:3],v),2.)
    return add(v,add(mul(t,q[3]),cross(q[:3],t)))

def bend_measure(p0,p1,p2):
    upper=sub(p1,p0);lower=sub(p2,p1);chord=sub(p2,p0)
    axis=unit(chord if length(chord)>1e-12 else upper)
    pole=project(upper,axis)
    valid=length(upper)>1e-12 and length(lower)>1e-12
    degrees=math.degrees(math.acos(max(-1.,min(1.,dot(unit(upper),unit(lower)))))) if valid else 0.
    return {'axis':axis,'pole':pole,'angle_degrees':degrees,
            'height_ratio':length(pole)/max(length(upper)+length(lower),1e-12),'valid':valid}

def transport_direction(reference_axis,reference_direction,current_axis):
    """Shortest-arc transport, with the stored pole resolving a 180deg chord."""
    a=unit(reference_axis);b=unit(current_axis);d=max(-1.,min(1.,dot(a,b)))
    direction=reference_direction
    if d<1.-1e-12:
        if d<-1.+1e-10:
            axis=unit(project(reference_direction,a));q=axis+[0.]
        else:
            q=cross(a,b)+[1.+d];q=mul(q,1./length(q))
        direction=rotate(reference_direction,q)
    return unit(project(direction,b))

def choose_direction(p0,p1,p2,reference_direction,previous_direction=None):
    """Use the measured plane only once bending is large enough to be reliable.

    Below 3deg use the reference (or a supplied previous valid plane); between
    3 and 8deg blend without allowing tiny opposite offsets to flip hemispheres.
    Above 8deg preserve the animated plane, including intentional reverse bends.
    Thresholds are angles and length ratios, never a fixed centimeter cutoff.
    """
    m=bend_measure(p0,p1,p2);axis=m['axis']
    fallback=unit(project(previous_direction if previous_direction is not None else reference_direction,axis))
    if length(fallback)<1e-10:fallback=unit(project(reference_direction,axis))
    known=length(fallback)>1e-10
    if not known:
        basis=min(([1.,0.,0.],[0.,1.,0.],[0.,0.,1.]),key=lambda v:abs(dot(v,axis)))
        fallback=unit(project(basis,axis))
    direction=fallback;source='reference'
    if m['valid'] and m['angle_degrees']>=RELIABLE_DEGREES and length(m['pole'])>1e-12:
        direction=unit(m['pole']);source='geometry';known=True
    elif m['valid'] and m['angle_degrees']>STRAIGHT_DEGREES and length(m['pole'])>1e-12:
        geometry=unit(m['pole'])
        if dot(geometry,fallback)<0.:geometry=mul(geometry,-1.)
        t=(m['angle_degrees']-STRAIGHT_DEGREES)/(RELIABLE_DEGREES-STRAIGHT_DEGREES)
        t=t*t*(3.-2.*t)
        direction=unit(add(mul(fallback,1.-t),mul(geometry,t)));source='blended'
    return {'direction':direction,'source':source,'orientation_known':known,
            'angle_degrees':m['angle_degrees'],'height_ratio':m['height_ratio']}

def unreal_angle_quaternion(angles):
    """UE Euler vector (Roll, Pitch, Yaw); Roll/Pitch have UE's negative sign."""
    r,p,y=[math.radians(v)*.5 for v in angles]
    sr,cr=math.sin(r),math.cos(r);sp,cp=math.sin(p),math.cos(p);sy,cy=math.sin(y),math.cos(y)
    return [cr*sp*sy-sr*cp*cy,-cr*sp*cy-sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]

def preferred_angle_direction(p0,p1,p2,joint_engine_rotation,angles):
    """Predict the pole from a small virtual bend in the solver's bone frame."""
    i=max(range(3),key=lambda j:abs(angles[j]))
    if abs(angles[i])<1e-10:return [0.,0.,0.]
    test=[0.,0.,0.];test[i]=math.copysign(5.,angles[i])
    delta=qmul(qmul(joint_engine_rotation,unreal_angle_quaternion(test)),inverse(joint_engine_rotation))
    virtual_end=add(p1,rotate(sub(p2,p1),delta))
    return unit(bend_measure(p0,p1,virtual_end)['pole'])

def calibrate_preferred_angles(p0,p1,p2,joint_engine_rotation,angles,desired_direction):
    predicted=preferred_angle_direction(p0,p1,p2,joint_engine_rotation,angles)
    alignment=dot(predicted,unit(desired_direction))
    flipped=length(predicted)>1e-10 and alignment<-.1
    return {'angles':mul(angles,-1.) if flipped else list(angles),
            'flipped':flipped,'predicted_alignment':alignment}

def select_triad(chain):
    bones=chain.bones
    if len(bones)<3:return None
    candidates=[b for b in bones[1:-1] if not any(t in chain.skeleton.get_clean_name(b).lower()
                for t in ('twist','roll','proc','helper','correct'))] or bones[1:-1]
    mid=mul(add(list(chain.skeleton.rest_world_pos[bones[0]]),list(chain.skeleton.rest_world_pos[bones[-1]])),.5)
    return bones[0],min(candidates,key=lambda b:length(sub(list(chain.skeleton.rest_world_pos[b]),mid))),bones[-1]

def settings_angles(settings):
    if 'preferred_angles' in settings:return list(settings['preferred_angles'])
    angles=[0.,0.,0.];axis=settings.get('preferred_axis','')
    if axis and axis[-1] in 'XYZ':angles['XYZ'.index(axis[-1])]=-90. if axis.startswith('-') else 90.
    return angles

def build_reference(chain,positions=None,rotations=None,settings=None):
    """Capture a pole and chord in the upper bone's frame for a biped limb."""
    triad=select_triad(chain)
    if triad is None:return None
    skel=chain.skeleton;positions=positions or skel.rest_world_pos;rotations=rotations or skel.rest_world_rot
    p0,p1,p2=[list(positions[n]) for n in triad];m=bend_measure(p0,p1,p2)
    direction=None;source='unresolved'
    if settings and settings.get('pole_direction') is not None:
        candidate=unit(project(settings['pole_direction'],m['axis']))
        if length(candidate)>1e-10:direction=candidate;source='explicit'
    if direction is None and m['angle_degrees']>=RELIABLE_DEGREES and length(m['pole'])>1e-12:
        direction=unit(m['pole']);source='bind_plane'
    if direction is None and ('leg' in chain.name.lower() or 'foot' in chain.name.lower()):
        end_index=skel.joint_indices[triad[2]]
        toes=[n for n in skel.joint_names if skel.parent_indices[skel.joint_indices[n]]==end_index
              and 'toe' in skel.get_clean_name(n).lower() and 'end' not in skel.get_clean_name(n).lower()]
        if toes:
            toe=min(toes,key=lambda n:length(sub(list(positions[n]),p2)))
            candidate=unit(project(sub(list(positions[toe]),p2),m['axis']))
            if length(candidate)>1e-10:direction=candidate;source='foot_to_toe'
    if direction is None and settings:
        # Import lazily so geometry-only tests do not require package/Maya setup.
        from .pbik_native import to_engine_pose
        points=[[p[0],p[2],p[1]] for p in (p0,p1,p2)]
        q=to_engine_pose(p1,list(rotations[triad[1]]))[3:7]
        predicted=preferred_angle_direction(*points,q,settings_angles(settings))
        candidate=[predicted[0],predicted[2],predicted[1]]
        if length(candidate)>1e-10:direction=candidate;source='preferred_angle'
    if direction is None:
        direction=choose_direction(p0,p1,p2,[0.,0.,0.])['direction']
    q=list(rotations[triad[0]])
    return {'bones':list(triad),'local_direction':rotate(direction,inverse(q)),
            'local_chord':rotate(m['axis'],inverse(q)),'source':source,
            'orientation_known':source!='unresolved','bind_angle_degrees':m['angle_degrees']}

def resolve_reference(reference,p0,p1,p2,root_rotation):
    q=list(root_rotation)
    direction=transport_direction(rotate(reference['local_chord'],q),rotate(reference['local_direction'],q),bend_measure(p0,p1,p2)['axis'])
    result=choose_direction(p0,p1,p2,direction)
    result['orientation_known']=result['source']=='geometry' or reference.get('orientation_known',True)
    return result

def calibrate_knee_settings(skeleton,chains,bone_settings,positions=None,rotations=None):
    """Copy settings, calibrating generated knee signs from actual bone frames."""
    from .pbik_native import to_engine_pose
    positions=positions or skeleton.rest_world_pos;rotations=rotations or skeleton.rest_world_rot
    result={name:dict(value) for name,value in bone_settings.items()}
    for name,chain in chains.items():
        if name not in ('LeftLeg','RightLeg'):continue
        triad=select_triad(chain)
        if triad is None:continue
        clean=skeleton.get_clean_name(triad[1]);settings=result.get(clean,{})
        if not settings.get('auto_pole_direction',True):continue
        reference=build_reference(chain,positions,rotations,settings=settings)
        if not reference['orientation_known']:continue
        p=[list(positions[n]) for n in triad]
        q=to_engine_pose(p[1],list(rotations[triad[1]]))[3:7]
        desired=rotate(reference['local_direction'],list(rotations[triad[0]]))
        angles=settings_angles(settings)
        if not any(angles):continue
        calibration=calibrate_preferred_angles(*[[v[0],v[2],v[1]] for v in p],q,angles,[desired[0],desired[2],desired[1]])
        previous=settings.get('pole_calibration',{})
        original=previous.get('source_angles',mul(angles,-1.) if previous.get('flipped') else list(angles))
        calibration['source_angles']=original
        calibration['calibrated']=any(abs(a-b)>1e-8 for a,b in zip(original,calibration['angles']))
        settings=dict(settings,preferred_angles=calibration['angles'],use_preferred_angles=True,
                      auto_pole_direction=True,pole_reference=reference,pole_calibration=calibration)
        i=max(range(3),key=lambda j:abs(settings['preferred_angles'][j]))
        settings['preferred_axis']=('+' if settings['preferred_angles'][i]>=0. else '-')+'XYZ'[i]
        result[clean]=settings
    return result
