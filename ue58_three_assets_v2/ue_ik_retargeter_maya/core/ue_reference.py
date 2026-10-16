"""Translate an explicitly exported UE RTG profile into a Maya cache bundle.

UE local offsets are conjugated through the verified FBX bone basis; they
cannot be treated as world offsets or converted by swapping Euler angles.
"""
import copy
import math
import maya.api.OpenMaya as om
from .pbik_native import multiply,from_engine_pose


def component_vector(v):return [v[0],v[2],v[1]]
def component_quaternion(q):return [-q[0],-q[2],-q[1],q[3]]
def local_quaternion(q):
    h=math.sqrt(.5)
    return component_quaternion(multiply(multiply([-h,0,0,h],q),[h,0,0,h]))


def _pose(rig, exported):
    world={};deltas={}
    bones=rig['skeleton']['bones']
    for i,bone in enumerate(bones):
        name=bone['dag_path'];parent=bone['parent_index']
        rest=om.MQuaternion(*bone['ref_world_rot'])
        rest_parent=om.MQuaternion(*bones[parent]['ref_world_rot']) if parent>=0 else om.MQuaternion.kIdentity
        local=(rest*rest_parent.inverse()).normal()
        parent_rotation=world[bones[parent]['dag_path']] if parent>=0 else om.MQuaternion.kIdentity
        propagated=(local*parent_rotation).normal()
        bone_name=bone['short_name'].split(':')[-1]
        delta=exported['local_rotation_offsets'].get(bone_name,[0.,0.,0.,1.])
        desired=(om.MQuaternion(*local_quaternion(delta))*local*parent_rotation).normal()
        world[name]=desired
        if max(abs(v) for v in delta[:3])>1e-10:
            q=(propagated.inverse()*desired).normal()
            deltas[name]={'bone':bone['clean_name'],'quat':list(q)}
    return {'root_translation_offset':component_vector(exported['root_translation_offset']),
        'bone_rotation_deltas_world':deltas,'ue_local_rotation_offsets':exported['local_rotation_offsets']}


def apply_ue_profile(bundle, profile):
    """Return an independent bundle using this UE profile's poses and settings."""
    result=copy.deepcopy(bundle);rtg=result['rtg']
    source=result['src_ikrig'];target=result['tgt_ikrig']
    # This option is specific to an explicitly supplied UE profile for the
    # same skeletons. Ordinary Maya caches keep their captured reference pose.
    for role,rig in [('Source',source),('Target',target)]:
        reference=profile.get('reference_poses',{}).get(role,{})
        bones=rig['skeleton']['bones']
        for bone in bones:
            name=bone['short_name'].split(':')[-1]
            if reference:
                if name not in reference:raise ValueError('Missing UE reference bone: '+name)
                e=reference[name];p,q,_=from_engine_pose(e['translation']+e['rotation']+e['scale3d'])
                bone['ref_world_pos']=p;bone['ref_world_rot']=list(om.MQuaternion(*q).normal())
        for bone in bones:
            parent=bone['parent_index'];p=om.MVector(*bone['ref_world_pos']);q=om.MQuaternion(*bone['ref_world_rot'])
            if parent>=0:
                pp=om.MVector(*bones[parent]['ref_world_pos']);pq=om.MQuaternion(*bones[parent]['ref_world_rot'])
                bone['ref_local_pos']=list((p-pp).rotateBy(pq.inverse()))
                bone['ref_local_rot']=list((q*pq.inverse()).normal())
            else:bone['ref_local_pos']=list(p);bone['ref_local_rot']=list(q)
    source_names={b['short_name'].split(':')[-1]:b['dag_path'] for b in source['skeleton']['bones']}
    target_names={b['short_name'].split(':')[-1]:b['dag_path'] for b in target['skeleton']['bones']}
    for role,rig in [('Source',source),('Target',target)]:
        name=rtg['current_'+role.lower()+'_retarget_pose']
        rtg[role.lower()+'_retarget_poses'][name]=_pose(rig,profile['poses'][role])
    by_type={o['type']:o for o in profile['ops']}
    pelvis=by_type['IKRetargetPelvisMotionController']
    settings=copy.deepcopy(pelvis['settings'])
    settings['blend_to_absolute']=component_vector(settings.pop('blend_to_absolute_offset'))
    for key in ['translation_offset_local','translation_offset_global','blend_to_source_translation_weights']:
        settings[key]=component_vector(settings[key])
    settings['rotation_offset_local']=local_quaternion(settings['rotation_offset_local'])
    settings['rotation_offset_global']=component_quaternion(settings['rotation_offset_global'])
    fk=by_type['IKRetargetFKChainsController'];ik=by_type['IKRetargetRunIKRigController']
    source_chains={c['chain_name']:c for c in profile['chains']['Source']}
    target_chains={c['chain_name']:c for c in profile['chains']['Target']}
    fk_settings={c['target_chain_name']:c for c in fk['chains']}
    ik_settings={c['target_chain_name']:c for c in ik['chains']}
    mappings=[]
    for entry in rtg['chain_mappings']:
        entry=copy.deepcopy(entry);chain=entry['target_chain'];source_chain=fk['mapping'].get(chain,'None')
        if source_chain=='None':continue
        source_definition=source_chains[source_chain];target_definition=target_chains[chain]
        entry['source_chain']=source_chain
        for role,definition,names in [('source',source_definition,source_names),('target',target_definition,target_names)]:
            for endpoint in ['start','end']:
                clean=definition[endpoint+'_bone']
                if clean not in names:raise ValueError('UE profile bone is missing: '+clean)
                entry[role+'_'+endpoint+'_bone']=clean
                entry[role+'_'+endpoint+'_dag']=names[clean]
        f=fk_settings[chain]
        entry.update({k:f[k] for k in ['enable_fk','rotation_alpha','translation_alpha']})
        entry['rotation_mode']={'INTERPOLATED':'Interpolated','ONE_TO_ONE':'OneToOne'}.get(f['rotation_mode'].split('.')[1].split(':')[0],f['rotation_mode'])
        entry['translation_mode']='None' if '.NONE:' in f['translation_mode'] else f['translation_mode']
        entry['enable_ik']=bool(ik_settings.get(chain,{}).get('enable_ik',False))
        entry['ik_goal_name']=target_definition['ik_goal_name'] if target_definition['ik_goal_name']!='None' else ''
        mappings.append(entry)
    rtg['chain_mappings']=mappings
    flags={'FIKRetargetPelvisMotionOp':pelvis['enabled'],'FIKRetargetFKChainsOp':fk['enabled'],
        'FIKRetargetIKChainsOp':ik['enabled'],'FIKRetargetRunIKRigOp':ik['enabled'],
        'FIKRetargetRootMotionOp':by_type['IKRetargetRootMotionController']['enabled'],
        'FIKRetargetAlignPoleVectorOp':False,'FIKRetargetPinBoneOp':False}
    for op in rtg['retarget_ops']:
        op['enabled']=flags.get(op['op_type'],False)
        if op['op_type']=='FIKRetargetPelvisMotionOp':op['settings'].update(settings)
        if op['op_type']=='FIKRetargetRunIKRigOp':op['settings']['ue_chain_settings']=ik_settings
    rtg['ue_reference']={'asset':profile['asset'],'engine_version':profile['engine_version']}
    return result
