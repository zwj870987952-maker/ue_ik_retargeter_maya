"""ctypes boundary for the standalone UE 5.8 PBIK core (no Maya dependency)."""
import ctypes as ct
import importlib.util
import math
from pathlib import Path

class BoneSettings(ct.Structure):
    _fields_=[('rotation_stiffness',ct.c_double),('position_stiffness',ct.c_double),
              ('limits',ct.c_int32*3),('minimum',ct.c_double*3),('maximum',ct.c_double*3),
              ('preferred',ct.c_double*3),('use_preferred',ct.c_int32)]

class SolverSettings(ct.Structure):
    _fields_=[('iterations',ct.c_int32),('sub_iterations',ct.c_int32),('mass',ct.c_double),
              ('root_behavior',ct.c_int32),('allow_stretch',ct.c_int32),
              ('pull_chain',ct.c_double),('max_angle',ct.c_double),('over_relaxation',ct.c_double),
              ('pre_pull',ct.c_double*8)]

class Goal(ct.Structure):
    _fields_=[('bone',ct.c_int32),('position',ct.c_double*3),('rotation',ct.c_double*4),
              ('position_alpha',ct.c_double),('rotation_alpha',ct.c_double),('strength',ct.c_double),
              ('pull_chain',ct.c_double),('pin_rotation',ct.c_double),('chain_depth',ct.c_int32)]

_LIBRARY=None

def library():
    global _LIBRARY
    if _LIBRARY is None:
        path=Path(__file__).resolve().parents[1]/'native/build.py'
        spec=importlib.util.spec_from_file_location('_ue58_pbik_build',str(path))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        dll=ct.CDLL(module.build())
        dll.pbik_abi.restype=ct.c_int32
        if dll.pbik_abi()!=1:raise RuntimeError('Unsupported PBIK native ABI.')
        dll.pbik_error.restype=ct.c_char_p
        dll.pbik_create.argtypes=[ct.POINTER(ct.c_int32),ct.c_int32,ct.c_int32,ct.POINTER(ct.c_double),ct.POINTER(ct.c_int32),ct.c_int32]
        dll.pbik_create.restype=ct.c_void_p
        dll.pbik_destroy.argtypes=[ct.c_void_p]
        dll.pbik_solve.argtypes=[ct.c_void_p,ct.POINTER(ct.c_double),ct.POINTER(BoneSettings),ct.POINTER(Goal),ct.POINTER(SolverSettings),ct.POINTER(ct.c_double)]
        dll.pbik_solve.restype=ct.c_int32
        _LIBRARY=dll
    return _LIBRARY

def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z]

def to_engine_pose(position,rotation,scale=(1,1,1)):
    # Component axes: UE=(Maya.x, Maya.z, Maya.y). Bone axes exported by
    # Maya FBX and imported by UE additionally differ by -90deg around bone X.
    # This right-hand bone-frame change was verified against both live assets.
    h=math.sqrt(.5)
    q=multiply([-rotation[0],-rotation[2],-rotation[1],rotation[3]],[-h,0,0,h])
    return [position[0],position[2],position[1]]+q+[scale[0],scale[2],scale[1]]

def from_engine_pose(values):
    h=math.sqrt(.5);q=multiply(values[3:7],[h,0,0,h])
    return ([values[0],values[2],values[1]],[-q[0],-q[2],-q[1],q[3]],
            [values[7],values[9],values[8]])

class NativePBIKSolver:
    def __init__(self,parents,root,reference,goals):
        self.dll=library();self.count=len(parents);self.goal_count=len(goals)
        self.pose_array=ct.c_double*(self.count*10)
        p=(ct.c_int32*self.count)(*parents)
        g=(ct.c_int32*self.goal_count)(*goals)
        flat=self.pose_array(*(v for pose in reference for v in pose))
        self.handle=self.dll.pbik_create(p,self.count,root,flat,g,self.goal_count)
        if not self.handle:raise RuntimeError(self.dll.pbik_error().decode())
        self.bones=(BoneSettings*self.count)()
        self.goals=(Goal*self.goal_count)()
        self.output=self.pose_array()

    def close(self):
        if getattr(self,'handle',None):self.dll.pbik_destroy(self.handle);self.handle=None

    def __del__(self):
        self.close()

    def solve(self,poses,bone_settings,goals,settings):
        if len(poses)!=self.count or len(goals)!=self.goal_count:raise ValueError('PBIK topology changed; recreate the solver.')
        for i,b in enumerate(bone_settings):
            s=self.bones[i];s.rotation_stiffness=max(0.,min(1.,b.get('rotation_stiffness',0.)));s.position_stiffness=max(0.,min(1.,b.get('position_stiffness',0.)))
            limits=b.get('limits',['Free']*3)
            s.limits[:]=[{'Free':0,'Limited':1,'Locked':2}.get(x,x) for x in limits]
            s.minimum[:]=b.get('minimum',[0]*3);s.maximum[:]=b.get('maximum',[0]*3)
            s.preferred[:]=b.get('preferred_angles',[0]*3);s.use_preferred=b.get('use_preferred_angles',False)
        for i,g in enumerate(goals):
            s=self.goals[i];s.bone=g['bone'];s.position[:]=g['position'];s.rotation[:]=g['rotation']
            s.position_alpha=g.get('position_alpha',1.);s.rotation_alpha=g.get('rotation_alpha',1.)
            s.strength=g.get('strength_alpha',1.);s.pull_chain=g.get('pull_chain_alpha',1.)
            s.pin_rotation=g.get('pin_rotation',1.);s.chain_depth=g.get('chain_depth',2)
        s=SolverSettings();s.iterations=settings.get('iterations',20);s.sub_iterations=settings.get('sub_iterations',10)
        s.mass=settings.get('mass_multiplier',1.);s.root_behavior={'PrePull':0,'PinToInput':1,'Free':2}[settings.get('root_behavior','Free')]
        s.allow_stretch=settings.get('allow_stretch',False);s.pull_chain=settings.get('global_pull_chain_alpha',0.)
        s.max_angle=settings.get('max_angle',30.);s.over_relaxation=settings.get('over_relaxation',1.3)
        s.pre_pull[:]=settings.get('pre_pull',[0,1,1,1,1,1,1,1])
        flat=self.pose_array(*(v for pose in poses for v in pose))
        if not self.dll.pbik_solve(self.handle,flat,self.bones,self.goals,ct.byref(s),self.output):raise RuntimeError(self.dll.pbik_error().decode())
        return [list(self.output[i*10:(i+1)*10]) for i in range(self.count)]
