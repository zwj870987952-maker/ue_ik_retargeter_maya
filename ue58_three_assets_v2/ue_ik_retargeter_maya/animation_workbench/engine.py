"""Maya-independent, integer-frame rhythm model. No SciPy or subprocess runtime."""
import bisect
import math

DEFAULTS = dict(hold_left=2, hold_right=3, hold_strength=100.,
                settle_duration=25., settle_amplitude=2., settle_strength=100.,
                prep_duration=25., prep_amplitude=3., prep_strength=100.,
                support_frames=22, rotate_timing=True)

def add(a,b): return [x+y for x,y in zip(a,b)]
def sub(a,b): return [x-y for x,y in zip(a,b)]
def mul(a,s): return [x*s for x in a]
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def length(a): return math.sqrt(dot(a,a))
def unit(a): return mul(a,1./max(length(a),1e-12))
def lerp(a,b,t): return add(a,mul(sub(b,a),t))
def clamp(v,a,b): return max(a,min(b,v))

def slerp(a,b,t):
    a=unit(a);b=unit(b);d=dot(a,b)
    if d<0.: b=mul(b,-1.);d=-d
    if d>.9995:return unit(lerp(a,b,t))
    theta=math.acos(clamp(d,-1.,1.));den=math.sin(theta)
    return unit(add(mul(a,math.sin((1.-t)*theta)/den),mul(b,math.sin(t*theta)/den)))

def quaternion_angle(a,b):
    return math.degrees(2.*math.acos(clamp(abs(dot(unit(a),unit(b))),0.,1.)))

def _hermite(a,b,da,db,u,span):
    return (2*u**3-3*u*u+1)*a+(u**3-2*u*u+u)*span*da+(-2*u**3+3*u*u)*b+(u**3-u*u)*span*db

class MonotoneCurve:
    def __init__(self,points):
        clean=[]
        for x,y in points:
            if clean and abs(x-clean[-1][0])<1e-9:
                if abs(y-clean[-1][1])>1e-8:raise ValueError('Inconsistent time-map knot')
                continue
            clean.append((float(x),float(y)))
        self.x=[p[0] for p in clean];self.y=[p[1] for p in clean]
        h=[b-a for a,b in zip(self.x,self.x[1:])]
        d=[(b-a)/s for a,b,s in zip(self.y,self.y[1:],h)]
        self.m=[0.]*len(clean)
        if not d:return
        self.m[0]=min(1.,3*d[0]);self.m[-1]=min(1.,3*d[-1])
        for i in range(1,len(clean)-1):
            if d[i-1]*d[i]<=0.:continue
            w1=2*h[i]+h[i-1];w2=h[i]+2*h[i-1]
            self.m[i]=(w1+w2)/(w1/d[i-1]+w2/d[i])
    def __call__(self,t):
        if len(self.x)==1:return self.y[0]
        i=max(0,min(len(self.x)-2,bisect.bisect_right(self.x,t)-1))
        h=self.x[i+1]-self.x[i];u=clamp((t-self.x[i])/h,0.,1.)
        return _hermite(self.y[i],self.y[i+1],self.m[i],self.m[i+1],u,h)

def sample(source,time):
    index=clamp(time-source['start'],0.,len(source['positions'])-1)
    i=min(len(source['positions'])-2,int(index));i=max(0,i);u=index-i
    p=lerp(source['positions'][i],source['positions'][min(i+1,len(source['positions'])-1)],u)
    q=slerp(source['rotations'][i],source['rotations'][min(i+1,len(source['rotations'])-1)],u)
    return p,q

def arc_length(source,a,b):
    times=[a]+list(range(int(math.ceil(a)),int(math.floor(b))+1))+[b]
    times=sorted(set(times));points=[sample(source,t)[0] for t in times]
    return sum(length(sub(b,a)) for a,b in zip(points,points[1:]))

def event_windows(markers,start,end,support=22):
    markers=sorted(markers,key=lambda m:int(m['frame']));events=[]
    frames=[int(m['frame']) for m in markers]
    if len(frames)!=len(set(frames)):raise ValueError('同一控制器的重点 pose 不能位于同一帧')
    for i,marker in enumerate(markers):
        b=int(marker['frame'])
        if not start<=b<=end:continue
        settings=dict(DEFAULTS);settings.update(marker.get('settings',{}))
        span=max(3,int(settings.get('support_frames',support)))
        a=max(start,b-span,(frames[i-1]+b)//2+1 if i else start)
        c=min(end,b+span,(frames[i+1]+b)//2-1 if i+1<len(frames) else end)
        a=min(a,b);c=max(c,b)
        left=min(max(0,int(settings['hold_left'])),max(0,b-a-1))
        right=min(max(0,int(settings['hold_right'])),max(0,c-b-1))
        hs=b-left;he=b+right
        duration_in=int(round((hs-a)*clamp(settings['settle_duration'],0.,100.)/100.))
        duration_out=int(round((c-he)*clamp(settings['prep_duration'],0.,100.)/100.))
        if settings['settle_duration']>0 and hs>a:duration_in=max(1,duration_in)
        if settings['prep_duration']>0 and c>he:duration_out=max(1,duration_out)
        events.append(dict(frame=b,a=a,c=c,hold_start=hs,hold_end=he,
                           arrival_start=hs-duration_in,departure_end=he+duration_out,
                           settings=settings,marker=marker.get('node','')))
    return events

def evaluate(source,markers,clip_start,clip_end,foot=False):
    """Evaluate complete segments; all output key times are integer frames.

    Duration % uses the available transition outside hold. Amplitude % uses
    the original transition's spatial arc length. Each effect has its own gain.
    """
    events=event_windows(markers,clip_start,clip_end)
    times=list(range(source['start'],source['start']+len(source['positions'])))
    result_p=[list(p) for p in source['positions']];result_q=[list(q) for q in source['rotations']]
    source_times=[float(t) for t in times]
    for event in events:
        a,b,c=event['a'],event['frame'],event['c'];hs,he=event['hold_start'],event['hold_end'];settings=event['settings']
        knots=[(a,a)]
        if hs>a:knots.extend([(a+(hs-a)*.34,a+(b-a)*.20),(hs,b)])
        else:knots.append((hs,b))
        knots.append((he,b))
        if c>he:knots.extend([(he+(c-he)*.60,b+(c-b)*.43),(c,c)])
        warp=MonotoneCurve(knots)
        hold_gain=clamp(settings['hold_strength'],0.,100.)/100.
        hold_enabled=hs!=he
        bp=sample(source,b)[0];incoming=unit(sub(bp,sample(source,max(a,b-5))[0]));outgoing=unit(sub(sample(source,min(c,b+5))[0],bp))
        lower=min(sample(source,t)[0][1] for t in range(a,c+1))
        contact=foot and bp[1]<=lower+2.
        if contact:
            incoming[1]=outgoing[1]=0.;incoming=unit(incoming);outgoing=unit(outgoing)
        in_amp=arc_length(source,a,b)*clamp(settings['settle_amplitude'],0.,100.)/100.*clamp(settings['settle_strength'],0.,100.)/100.
        out_amp=arc_length(source,b,c)*clamp(settings['prep_amplitude'],0.,100.)/100.*clamp(settings['prep_strength'],0.,100.)/100.
        event.update(contact=contact,floor_y=lower,arrival_cm=in_amp,prep_cm=out_amp)
        for t in range(a,c+1):
            index=t-source['start']
            tau=t+(warp(t)-t)*hold_gain if hold_enabled else float(t)
            p,q=sample(source,tau)
            # Explicitly exclude the entire hold interval, including its endpoints.
            if event['arrival_start']<t<hs:
                u=(t-event['arrival_start'])/float(hs-event['arrival_start']);pulse=16*u*u*(1-u)*(1-u)
                p=add(p,mul(incoming,in_amp*pulse))
            if he<t<event['departure_end']:
                u=(t-he)/float(event['departure_end']-he);pulse=16*u*u*(1-u)*(1-u)
                p=sub(p,mul(outgoing,out_amp*pulse))
            if contact:p[1]=max(p[1],lower)
            result_p[index]=p;result_q[index]=q if settings.get('rotate_timing',True) else list(source['rotations'][index]);source_times[index]=tau
    return dict(times=times,positions=result_p,rotations=result_q,source_times=source_times,events=events)

def detect(source,start,end,min_gap=30,max_count=8):
    """Deterministic code-only slowdown, reversal and spatial-extremum detection."""
    first=source['start'];p=source['positions'];n=len(p)
    smooth=[]
    for i in range(n):
        neighbours=[(p[j],3-abs(j-i)) for j in range(max(0,i-2),min(n,i+3))]
        total=sum(w for _,w in neighbours);smooth.append([sum(v[k]*w for v,w in neighbours)/total for k in range(3)])
    speed=[length(sub(smooth[min(n-1,i+1)],smooth[max(0,i-1)]))*.5 for i in range(n)]
    high=max(sorted(speed)[int(.85*(n-1))],.001)
    axis_range=[max(row[k] for row in p)-min(row[k] for row in p) for k in range(3)]
    candidates=[]
    for i in range(max(3,start-first+3),min(n-3,end-first-2)):
        before=sub(smooth[i],smooth[max(0,i-4)]);after=sub(smooth[min(n-1,i+4)],smooth[i]);den=length(before)*length(after)
        turn=math.acos(clamp(dot(before,after)/den,-1.,1.))/math.pi if den>1e-8 else 0.
        low=speed[i]<=speed[i-1] and speed[i]<=speed[i+1]
        extrema=0
        for k in range(3):
            if (smooth[i][k]-smooth[i-1][k])*(smooth[i+1][k]-smooth[i][k])<=0.:
                left=smooth[max(0,i-9)][k];right=smooth[min(n-1,i+9)][k]
                if min(abs(smooth[i][k]-left),abs(smooth[i][k]-right))>max(.15,axis_range[k]*.025):extrema+=1
        if not low and not extrema:continue
        moving=sum(length(sub(p[j+1],p[j])) for j in range(max(0,i-12),min(n-1,i+12)))
        if moving<.5:continue
        score=(1.-min(speed[i]/high,1.))*1.5+turn*2.+extrema*.7+(1. if low else 0.)
        candidates.append((score,i+first))
    chosen=[]
    for score,frame in sorted(candidates,reverse=True):
        if all(abs(frame-other)>=min_gap for other in chosen):chosen.append(frame)
        if len(chosen)>=max_count:break
    return sorted(chosen)


def layer_components(source,markers,start,end,foot=False):
    """Bake full-strength targets once; Maya layers supply live mixing gains."""
    def points(settle,prep):
        result=[]
        for marker in markers:
            m=dict(marker);values=dict(marker.get('settings',{}))
            values.update(hold_strength=100.,settle_strength=settle,prep_strength=prep)
            m['settings']=values;result.append(m)
        return result
    hold=evaluate(source,points(0.,0.),start,end,foot)
    settle=evaluate(source,points(100.,0.),start,end,foot)
    prep=evaluate(source,points(0.,100.),start,end,foot)
    return dict(hold=hold,
                settle=[sub(a,b) for a,b in zip(settle['positions'],hold['positions'])],
                prep=[sub(a,b) for a,b in zip(prep['positions'],hold['positions'])])
