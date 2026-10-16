"""Maya API 2.0 dependency node for the standalone UE 5.8 PBIK core.

Input joint.matrix values contain animation / jointOrient but exclude
offsetParentMatrix. Output offsets drive the evaluated pose without writing
translate / rotate, disconnecting animation, or introducing a worldMatrix loop.
"""
import json
import time
import maya.api.OpenMaya as om

def maya_useNewAPI():
    pass

class UE58PBIKNode(om.MPxNode):
    TYPE_NAME='ue58PBIKSolver'
    TYPE_ID=om.MTypeId(0x0013B2A1)
    attributes={}

    def __init__(self):
        super(UE58PBIKNode,self).__init__()
        self.configuration_text=None
        self.config=None
        self.native=None
        self.signature=None

    @staticmethod
    def creator():
        return UE58PBIKNode()

    @classmethod
    def initialize(cls):
        inputs=[]
        def add(name,fn,obj,array=False,output=False):
            fn.array=array
            if array:fn.usesArrayDataBuilder=True
            if output:fn.writable=False;fn.storable=False
            cls.addAttribute(obj);cls.attributes[name]=obj
            if not output and name!='pelvisMatrix':inputs.append(obj)
            return obj
        t=om.MFnTypedAttribute();add('configuration',t,t.create('configuration','cfg',om.MFnData.kString))
        for name,short,array,output in [('inputLocalMatrix','ilm',True,False),('goalMatrix','gm',True,False),
                ('externalParentMatrix','epm',False,False),('pelvisMatrix','pm',False,False),('outputOffset','oo',True,True)]:
            f=om.MFnMatrixAttribute();add(name,f,f.create(name,short),array,output)
            # Read compatibility for older scenes. This former control input
            # is hidden and never participates in solving or dirty propagation.
            if name=='pelvisMatrix':f.hidden=True
        for name,short,kind,default,array in [
                ('enabled','en',om.MFnNumericData.kBoolean,True,False),
                ('iterations','it',om.MFnNumericData.kInt,20,False),('subIterations','si',om.MFnNumericData.kInt,10,False),
                ('rootMode','rm',om.MFnNumericData.kInt,2,False),('allowStretch','ast',om.MFnNumericData.kBoolean,False,False),
                ('massMultiplier','mm',om.MFnNumericData.kDouble,1.,False),('globalPullChainAlpha','gpca',om.MFnNumericData.kDouble,0.,False),
                ('maxAngle','ma',om.MFnNumericData.kDouble,30.,False),('overRelaxation','orx',om.MFnNumericData.kDouble,1.3,False),
                ('positionAlpha','pa',om.MFnNumericData.kDouble,1.,True),('rotationAlpha','ra',om.MFnNumericData.kDouble,1.,True),
                ('strengthAlpha','sa',om.MFnNumericData.kDouble,1.,True),('pullChainAlpha','pca',om.MFnNumericData.kDouble,1.,True),
                ('pinRotation','pr',om.MFnNumericData.kDouble,1.,True),('chainDepth','cd',om.MFnNumericData.kInt,2,True),
                ('previewMode','pvm',om.MFnNumericData.kInt,0,True),
                ('boneRotationStiffness','brs',om.MFnNumericData.kDouble,0.,True),
                ('bonePositionStiffness','bps',om.MFnNumericData.kDouble,0.,True),
                ('boneUsePreferredAngles','bupa',om.MFnNumericData.kBoolean,False,True),
                ('boneExcluded','bex',om.MFnNumericData.kBoolean,False,True)]:
            f=om.MFnNumericAttribute();add(name,f,f.create(name,short,kind,default),array)
        for name,short,kind in [('boneLimits','bl',om.MFnNumericData.k3Int),('boneMinimum','bmin',om.MFnNumericData.k3Double),
                               ('boneMaximum','bmax',om.MFnNumericData.k3Double),('bonePreferredAngles','bpa',om.MFnNumericData.k3Double)]:
            f=om.MFnNumericAttribute();add(name,f,f.create(name,short,kind),True)
        f=om.MFnUnitAttribute();add('inputTime',f,f.create('inputTime','tm',om.MFnUnitAttribute.kTime,0.))
        f=om.MFnNumericAttribute();add('solveMilliseconds',f,f.create('solveMilliseconds','sms',om.MFnNumericData.kDouble,0.),False,True)
        for a in inputs:
            cls.attributeAffects(a,cls.attributes['outputOffset'])
            cls.attributeAffects(a,cls.attributes['solveMilliseconds'])

    def compute(self,plug,data):
        a=self.attributes
        attr=plug.attribute()
        if attr not in (a['outputOffset'],a['solveMilliseconds']):
            # API 2.0 delegates inherited/unsupported plugs by returning None.
            # kUnknownParameter is a C++/API 1.0 status, not an API 2.0 symbol.
            return None
        from ue_ik_retargeter_maya.core.pbik_native import NativePBIKSolver,to_engine_pose,from_engine_pose
        from ue_ik_retargeter_maya.core.math_utils import matrix_to_trs,trs_to_matrix,slerp
        started=time.perf_counter()
        text=data.inputValue(a['configuration']).asString()
        if not text:return
        if text!=self.configuration_text:
            self.config=json.loads(text);self.configuration_text=text;self.signature=None
        cfg=self.config
        if 'retarget_bundle' in cfg:
            from ue_ik_retargeter_maya.core.pose_evaluator import compute_cached_node
            return compute_cached_node(self,plug,data)
        count=len(cfg['parents'])
        def scalar(name,kind='Double'):
            return getattr(data.inputValue(a[name]),'as'+kind)()
        def array(name,index,kind='Double',default=0.):
            h=data.inputArrayValue(a[name])
            try:h.jumpToLogicalElement(index)
            except RuntimeError:return default
            return getattr(h.inputValue(),'as'+kind)()
        baseline=[om.MMatrix(v) for v in cfg['baseline_offsets']]
        local=[array('inputLocalMatrix',i,'Matrix',om.MMatrix()) for i in range(count)]
        external=data.inputValue(a['externalParentMatrix']).asMatrix()
        input_world=[]
        for i,parent in enumerate(cfg['parents']):
            m=local[i]*baseline[i]
            delta=cfg['alignment_rotations'][i]
            if delta is not None:
                p,q,s=matrix_to_trs(m);q=(q*om.MQuaternion(*delta)).normal();m=trs_to_matrix(p,q,s)
            input_world.append(m*(input_world[parent] if parent>=0 else external))
        if not scalar('enabled','Bool'):
            offsets=baseline
        else:
            unmodified=list(input_world)
            pelvis=cfg['root']
            excluded=tuple(i for i in range(count) if array('boneExcluded',i,'Bool',False))
            parents=list(cfg['parents'])
            for i in range(count):
                while parents[i] in excluded:parents[i]=parents[parents[i]]
            signature=(text,excluded)
            if signature!=self.signature:
                if self.native:self.native.close()
                self.native=NativePBIKSolver(parents,pelvis,cfg['native_reference'],[g['bone'] for g in cfg['goals']])
                self.signature=signature
            poses=[]
            for m in input_world:
                p,q,s=matrix_to_trs(m);poses.append(to_engine_pose([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],[s.x,s.y,s.z]))
            bones=[]
            for i in range(count):
                bones.append({'rotation_stiffness':array('boneRotationStiffness',i),
                    'position_stiffness':array('bonePositionStiffness',i),'limits':array('boneLimits',i,'Int3',(0,0,0)),
                    'minimum':array('boneMinimum',i,'Double3',(0.,0.,0.)),
                    'maximum':array('boneMaximum',i,'Double3',(0.,0.,0.)),
                    'preferred_angles':array('bonePreferredAngles',i,'Double3',(0.,0.,0.)),
                    'use_preferred_angles':array('boneUsePreferredAngles',i,'Bool',False)})
            goals=[]
            for i,g in enumerate(cfg['goals']):
                gp,gq,_=matrix_to_trs(array('goalMatrix',i,'Matrix',om.MMatrix()))
                ip,iq,_=matrix_to_trs(unmodified[g['bone']])
                mode=array('previewMode',i,'Int',0)
                if mode==0:
                    initial_p=om.MVector(*g['initial_position']);initial_q=om.MQuaternion(*g['initial_rotation'])
                    if 'initial_external_parent_matrix' in cfg:
                        initial=trs_to_matrix(initial_p,initial_q)*om.MMatrix(cfg['initial_external_parent_matrix']).inverse()*external
                        initial_p,initial_q,_=matrix_to_trs(initial)
                    gp=ip+(gp-initial_p)
                    gq=(iq*initial_q.inverse()*gq).normal()
                pa=max(0.,min(1.,array('positionAlpha',i,default=1.)))
                ra=max(0.,min(1.,array('rotationAlpha',i,default=1.)))
                gp=ip+(gp-ip)*pa;gq=slerp(iq,gq,ra).normal()
                native=to_engine_pose([gp.x,gp.y,gp.z],[gq.x,gq.y,gq.z,gq.w])
                goals.append({'bone':g['bone'],'position':native[:3],'rotation':native[3:7],
                    'strength_alpha':array('strengthAlpha',i,default=1.),'pin_rotation':array('pinRotation',i,default=1.),
                    'pull_chain_alpha':array('pullChainAlpha',i,default=1.),'chain_depth':array('chainDepth',i,'Int',2)})
            root_mode=max(0,min(2,scalar('rootMode','Int')))
            settings={'iterations':max(0,scalar('iterations','Int')),'sub_iterations':max(0,scalar('subIterations','Int')),
                'root_behavior':['PrePull','PinToInput','Free'][root_mode], 'allow_stretch':scalar('allowStretch','Bool'),
                'mass_multiplier':max(0.,scalar('massMultiplier')),'global_pull_chain_alpha':scalar('globalPullChainAlpha'),
                'max_angle':max(0.,min(180.,scalar('maxAngle'))),'over_relaxation':max(1.,min(2.,scalar('overRelaxation')))}
            solved=self.native.solve(poses,bones,goals,settings)
            if cfg.get('leg_planes'):
                from ue_ik_retargeter_maya.core.pole_vector import stabilize_limb_planes
                solved=stabilize_limb_planes(poses,solved,cfg['leg_planes'],cfg.get('leg_plane_strength',1.))
            solved_world=[]
            for pose in solved:
                p,q,s=from_engine_pose(pose);solved_world.append(trs_to_matrix(om.MVector(*p),om.MQuaternion(*q),om.MVector(*s)))
            offsets=[]
            for i,parent in enumerate(cfg['parents']):
                # Bone matrices use Maya's row convention: local * offset * parent.
                parent_world=solved_world[parent] if parent>=0 else external
                offsets.append(local[i].inverse()*solved_world[i]*parent_world.inverse())
        out=data.outputArrayValue(a['outputOffset']);builder=out.builder()
        for i,m in enumerate(offsets):builder.addElement(i).setMMatrix(m)
        out.set(builder);out.setAllClean()
        data.outputValue(a['solveMilliseconds']).setDouble((time.perf_counter()-started)*1000.)
        data.setClean(plug)

def initializePlugin(plugin):
    fn=om.MFnPlugin(plugin,'UE58 IK Retargeter','2.0','Any')
    fn.registerNode(UE58PBIKNode.TYPE_NAME,UE58PBIKNode.TYPE_ID,UE58PBIKNode.creator,UE58PBIKNode.initialize)

def uninitializePlugin(plugin):
    om.MFnPlugin(plugin).deregisterNode(UE58PBIKNode.TYPE_ID)
