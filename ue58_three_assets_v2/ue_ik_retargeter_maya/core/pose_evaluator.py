"""Frame-independent retarget pose evaluation for cached A/B rigs.

No scene writes or world-transform queries: the DG node supplies source poses
and settings, and receives one complete target pose.
"""
import maya.api.OpenMaya as om
from .chains import BoneChain, ChainMapping
from .retarget_pose import RetargetPoseManager
from .pelvis_solver import PelvisMotionSolver
from .fk_solver import FKChainSolver
from .ik_solver import IKChainSolver, AlignPoleVectorSolver
from .pole_vector import select_triad
from .pbik_native import NativePBIKSolver, to_engine_pose, from_engine_pose
from .math_utils import rotate_vector_by_quat, slerp


class PoseSkeleton:
    def __init__(self, rig):
        entries=rig['skeleton']['bones']
        self.joint_names=[e['dag_path'] for e in entries]
        self.joint_indices={n:i for i,n in enumerate(self.joint_names)}
        self.parent_indices=[e['parent_index'] for e in entries]
        self.root_joint=self.joint_names[0]
        self.clean_names={e['dag_path']:e['clean_name'] for e in entries}
        self.clean_to_dag={v:k for k,v in self.clean_names.items()}
        self.rest_world_pos={e['dag_path']:om.MVector(*e['ref_world_pos']) for e in entries}
        self.rest_world_rot={e['dag_path']:om.MQuaternion(*e['ref_world_rot']) for e in entries}
        self.rest_local_pos={e['dag_path']:om.MVector(*e['ref_local_pos']) for e in entries}
        self.rest_local_rot={e['dag_path']:om.MQuaternion(*e['ref_local_rot']) for e in entries}

    def resolve_bone_dag(self, name):
        if name in self.joint_indices:return name
        return self.clean_to_dag.get(name)

    def get_clean_name(self, name):return self.clean_names.get(name,name)
    def get_bone_index(self, name):return self.joint_indices.get(self.resolve_bone_dag(name),-1)
    def get_parent_joint(self, name):
        i=self.get_bone_index(name)
        return self.joint_names[self.parent_indices[i]] if i>=0 and self.parent_indices[i]>=0 else None
    def get_children_indices_recursive(self, start):
        result=[];visited={start}
        for i in range(start+1,len(self.joint_names)):
            if self.parent_indices[i] in visited:visited.add(i);result.append(i)
        return result


class PoseChain(BoneChain):
    def _build_chain(self, custom_pos_dict=None):
        chain=[self.end_bone]
        while chain[-1]!=self.start_bone:
            parent=self.skeleton.get_parent_joint(chain[-1])
            if parent is None:raise ValueError('Invalid cached chain: '+self.name)
            chain.append(parent)
        self.bones=list(reversed(chain))
        self.clean_bones=[self.skeleton.get_clean_name(b) for b in self.bones]
        positions=custom_pos_dict or self.skeleton.rest_world_pos
        self.initial_lengths=[0.]
        for a,b in zip(self.bones,self.bones[1:]):
            self.initial_lengths.append(self.initial_lengths[-1]+(positions[b]-positions[a]).length())
        self.total_initial_length=max(1.e-4,self.initial_lengths[-1])
        self.params=[v/self.total_initial_length for v in self.initial_lengths] if len(self.bones)>1 else [1.]


class CachedPoseEvaluator:
    def __init__(self, bundle, configuration):
        self.source=PoseSkeleton(bundle['src_ikrig']);self.target=PoseSkeleton(bundle['tgt_ikrig'])
        self.source_pose=RetargetPoseManager(self.source);self.target_pose=RetargetPoseManager(self.target)
        rtg=bundle['rtg']
        source_data=rtg['source_retarget_poses'][rtg['current_source_retarget_pose']]
        self.source_pose.root_translation_offset=om.MVector(*source_data.get('root_translation_offset',[0,0,0]))
        for n,e in source_data.get('bone_rotation_deltas_world',{}).items():
            if n in self.source.joint_indices:self.source_pose.delta_rotations[n]=om.MQuaternion(*e['quat'])
        self.source_pose.recompute_aligned_transforms()
        pose_data=rtg['target_retarget_poses'][rtg['current_target_retarget_pose']]
        self.target_pose.root_translation_offset=om.MVector(*pose_data.get('root_translation_offset',[0,0,0]))
        for n,e in pose_data.get('bone_rotation_deltas_world',{}).items():
            if n in self.target.joint_indices:self.target_pose.delta_rotations[n]=om.MQuaternion(*e['quat'])
        self.target_pose.recompute_aligned_transforms()
        self.source_pelvis=bundle['src_ikrig']['skeleton']['pelvis_bone_dag']
        self.target_pelvis=bundle['tgt_ikrig']['skeleton']['pelvis_bone_dag']
        self.pelvis=PelvisMotionSolver(self.source_pelvis,self.target_pelvis,self.source,self.target,self.target_pose)
        self.ops={op['op_type']:op for op in rtg['retarget_ops']}
        for key,value in self.ops['FIKRetargetPelvisMotionOp']['settings'].items():
            if key in ['height_ratio','height_scale']:continue  # derived from reference poses, not an Op setting
            if hasattr(self.pelvis,key) and isinstance(value,(int,float,bool,list,tuple)):setattr(self.pelvis,key,value)
        self.fk=[];self.ik=[]
        for e in sorted(rtg['chain_mappings'],key=lambda e:self.target.get_bone_index(e['target_start_dag'])):
            src=PoseChain(e['source_chain'],e['source_start_dag'],e['source_end_dag'],self.source)
            tgt=PoseChain(e['target_chain'],e['target_start_dag'],e['target_end_dag'],self.target,e.get('ik_goal_name',''))
            mapping=ChainMapping(e['target_chain'],src,tgt,e.get('enable_ik',False),e.get('ik_goal_name',''))
            for key in ['enable_fk','enable_ik','rotation_mode','rotation_alpha','translation_mode','translation_alpha']:
                if key in e:setattr(mapping,key,e[key])
            if mapping.rotation_mode not in ['Interpolated','OneToOne'] or mapping.translation_mode!='None':
                raise ValueError('Unsupported cached FK mode for '+mapping.name)
            self.fk.append(FKChainSolver(mapping,self.target_pose,self.source_pose))
            if mapping.enable_ik and len(tgt.bones)>=3:self.ik.append(IKChainSolver(mapping,self.target_pose,self.source_pose))
        self.config=configuration;self.native=None;self.native_signature=None
        self.last_native_solves=0

    def close(self):
        if self.native:self.native.close();self.native=None

    def enabled(self, name):return bool(self.ops.get(name,{}).get('enabled',False))

    def _propagate(self, start, positions, rotations, locals_):
        index=self.target.get_bone_index(start);parent=self.target.parent_indices[index]
        if parent>=0:
            p=self.target.joint_names[parent]
            locals_[start]=(rotations[start]*rotations[p].inverse()).normal()
            positions[start]=positions[p]+rotate_vector_by_quat(self.target_pose.aligned_local_pos[start],rotations[p])
        else:locals_[start]=om.MQuaternion(rotations[start])
        for child in self.target.get_children_indices_recursive(index):
            name=self.target.joint_names[child];p=self.target.joint_names[self.target.parent_indices[child]]
            positions[name]=positions[p]+rotate_vector_by_quat(self.target_pose.aligned_local_pos[name],rotations[p])
            rotations[name]=(locals_[name]*rotations[p]).normal()

    def evaluate(self, source_pose, bones, goal_settings, solver_settings, run_ik=True, align_poles=True):
        self.last_native_solves=0
        positions={n:om.MVector(self.target_pose.get_aligned_rest_position(n)) for n in self.target.joint_names}
        rotations={n:om.MQuaternion(self.target_pose.get_aligned_rest_rotation(n)) for n in self.target.joint_names}
        locals_={n:om.MQuaternion(q) for n,q in self.target_pose.aligned_local_rot.items()}
        if self.enabled('FIKRetargetPelvisMotionOp'):
            positions[self.target_pelvis],rotations[self.target_pelvis]=self.pelvis.solve(*source_pose[self.source_pelvis][:2])
            self._propagate(self.target_pelvis,positions,rotations,locals_)
        pelvis_delta=self.pelvis.pelvis_translation_delta if self.enabled('FIKRetargetPelvisMotionOp') else om.MVector()
        if self.enabled('FIKRetargetFKChainsOp'):
            for fk in self.fk:
                if not fk.mapping.enable_fk:continue
                parent=self.target.get_parent_joint(fk.tgt_chain.start_bone)
                view={n:(positions[n],rotations[n]) for n in fk.tgt_chain.bones}
                if parent:view[parent]=(positions[parent],rotations[parent])
                result=fk.solve(source_pose,view)
                for n in fk.tgt_chain.bones:
                    rotations[n]=result[n];self._propagate(n,positions,rotations,locals_)
        if run_ik and self.enabled('FIKRetargetRunIKRigOp'):
            goals=[];chain_goals={s.mapping.ik_goal_name:s for s in self.ik if s.mapping.enable_ik}
            for index,g in enumerate(self.config['goals']):
                name=self.target.joint_names[g['bone']];p=positions[name];q=rotations[name]
                ik=chain_goals.get(g['name'])
                if ik and self.enabled('FIKRetargetIKChainsOp'):
                    p,q=ik.calculate_goal_transform(source_pose,positions[ik.tgt_chain.start_bone],pelvis_delta)
                    inv_weight=om.MVector(1.-self.pelvis.affect_ik_horizontal,1.-self.pelvis.affect_ik_vertical,1.-self.pelvis.affect_ik_horizontal)
                    p-=om.MVector(pelvis_delta.x*inv_weight.x,pelvis_delta.y*inv_weight.y,pelvis_delta.z*inv_weight.z)
                settings=goal_settings[index]
                p+=settings.get('offset_position',om.MVector())
                q=(q*settings.get('offset_rotation',om.MQuaternion.kIdentity)).normal()
                p=positions[name]+(p-positions[name])*max(0.,min(1.,settings.get('position_alpha',1.)))
                q=slerp(rotations[name],q,max(0.,min(1.,settings.get('rotation_alpha',1.)))).normal()
                pose=to_engine_pose(list(p),list(q))
                goal=dict(settings,bone=g['bone'],position=pose[:3],rotation=pose[3:7],position_alpha=1.,rotation_alpha=1.)
                goals.append(goal)
            parents=list(self.target.parent_indices)
            excluded={i for i,b in enumerate(bones) if b.get('excluded',False)}
            for i in range(len(parents)):
                while parents[i] in excluded:parents[i]=parents[parents[i]]
            signature=tuple(parents)
            if self.native is None or signature!=self.native_signature:
                self.close();self.native=NativePBIKSolver(parents,self.config['root'],self.config['native_reference'],[g['bone'] for g in goals]);self.native_signature=signature
            current=[to_engine_pose(list(positions[n]),list(rotations[n])) for n in self.target.joint_names]
            solved=self.native.solve(current,bones,goals,solver_settings);self.last_native_solves=1
            for n,pose in zip(self.target.joint_names,solved):
                p,q,_=from_engine_pose(pose);positions[n]=om.MVector(*p);rotations[n]=om.MQuaternion(*q)
            for n in self.target.joint_names:
                parent=self.target.get_parent_joint(n)
                locals_[n]=(rotations[n]*rotations[parent].inverse()).normal() if parent else om.MQuaternion(rotations[n])
        if align_poles and self.enabled('FIKRetargetAlignPoleVectorOp'):
            for ik in self.ik:
                target=select_triad(ik.tgt_chain);source=select_triad(ik.src_chain)
                if target is None or source is None:continue
                a,b,c=target
                new_a,new_b,_,roll=AlignPoleVectorSolver.align_limb_pole_vector(
                    *[source_pose[n][0] for n in source],positions[a],positions[b],positions[c],rotations[a],rotations[b],
                    src_reference_direction=ik.pole_direction(source_pose,source=True),
                    tgt_reference_direction=ik.pole_direction({n:(positions[n],rotations[n]) for n in ik.tgt_chain.bones},source=False))
                if abs(roll.w)<.999999:
                    rotations[a]=new_a;self._propagate(a,positions,rotations,locals_)
                    rotations[b]=new_b;self._propagate(b,positions,rotations,locals_)
        return [(positions[n],rotations[n],om.MVector(1,1,1)) for n in self.target.joint_names]


def compute_cached_node(node, plug, data):
    """Use the existing PBIK node schema for a whole cached retarget pipeline.

    inputLocalMatrix contains source world matrices followed by target local
    matrices. goalMatrix[goal_count] is the source display parent's matrix.
    This uses the existing registered node type, including in Maya sessions
    where the plugin is already loaded; no new attribute schema is required.
    """
    import time
    from .math_utils import matrix_to_trs,trs_to_matrix
    started=time.perf_counter();a=node.attributes;cfg=node.config
    def scalar(name,kind='Double'):return getattr(data.inputValue(a[name]),'as'+kind)()
    def array(name,index,kind='Double',default=0.):
        h=data.inputArrayValue(a[name])
        try:h.jumpToLogicalElement(index)
        except RuntimeError:return default
        return getattr(h.inputValue(),'as'+kind)()
    source_count=cfg['source_count'];count=len(cfg['parents'])
    source_display=array('goalMatrix',len(cfg['goals']),'Matrix',om.MMatrix())
    target_display=data.inputValue(a['externalParentMatrix']).asMatrix()
    if getattr(node,'retarget_runtime',None) is None or getattr(node,'retarget_signature',None)!=node.configuration_text:
        runtime=getattr(node,'retarget_runtime',None)
        if runtime:runtime.close()
        node.retarget_runtime=CachedPoseEvaluator(cfg['retarget_bundle'],cfg)
        node.retarget_signature=node.configuration_text
    runtime=node.retarget_runtime
    source={n:matrix_to_trs(array('inputLocalMatrix',i,'Matrix',om.MMatrix())*source_display.inverse())[:2]
            for i,n in enumerate(runtime.source.joint_names)}
    bones=[]
    for i in range(count):
        bones.append({'rotation_stiffness':array('boneRotationStiffness',i),'position_stiffness':array('bonePositionStiffness',i),
            'limits':array('boneLimits',i,'Int3',(0,0,0)),'minimum':array('boneMinimum',i,'Double3',(0.,0.,0.)),
            'maximum':array('boneMaximum',i,'Double3',(0.,0.,0.)),
            'preferred_angles':array('bonePreferredAngles',i,'Double3',(0.,0.,0.)),
            'use_preferred_angles':array('boneUsePreferredAngles',i,'Bool',False),'excluded':array('boneExcluded',i,'Bool',False)})
    goals=[]
    for i,g in enumerate(cfg['goals']):
        p,q,_=matrix_to_trs(array('goalMatrix',i,'Matrix',om.MMatrix())*target_display.inverse())
        goals.append({'offset_position':p-om.MVector(*g['initial_position']),
            'offset_rotation':(om.MQuaternion(*g['initial_rotation']).inverse()*q).normal(),
            'position_alpha':array('positionAlpha',i,default=1.),'rotation_alpha':array('rotationAlpha',i,default=1.),
            'strength_alpha':array('strengthAlpha',i,default=1.),'pull_chain_alpha':array('pullChainAlpha',i,default=1.),
            'pin_rotation':array('pinRotation',i,default=1.),'chain_depth':array('chainDepth',i,'Int',2)})
    settings={'iterations':max(0,scalar('iterations','Int')),'sub_iterations':max(0,scalar('subIterations','Int')),
        'root_behavior':['PrePull','PinToInput','Free'][max(0,min(2,scalar('rootMode','Int')))],
        'allow_stretch':scalar('allowStretch','Bool'),'mass_multiplier':max(0.,scalar('massMultiplier')),
        'global_pull_chain_alpha':scalar('globalPullChainAlpha'),'max_angle':scalar('maxAngle'),'over_relaxation':scalar('overRelaxation')}
    result=runtime.evaluate(source,bones,goals,settings,run_ik=scalar('enabled','Bool'),align_poles=cfg.get('align_poles',False))
    world=[trs_to_matrix(*pose)*target_display for pose in result]
    out=data.outputArrayValue(a['outputOffset']);builder=out.builder()
    for i,parent in enumerate(cfg['parents']):
        local=array('inputLocalMatrix',source_count+i,'Matrix',om.MMatrix())
        parent_world=world[parent] if parent>=0 else target_display
        builder.addElement(i).setMMatrix(local.inverse()*world[i]*parent_world.inverse())
    out.set(builder);out.setAllClean()
    data.outputValue(a['solveMilliseconds']).setDouble((time.perf_counter()-started)*1000.)
    data.setClean(plug)
