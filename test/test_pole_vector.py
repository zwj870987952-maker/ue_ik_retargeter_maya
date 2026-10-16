"""Scale, straight-limb and anatomical-sign regressions without Maya."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('pole_vector',ROOT/'ue58_three_assets_v2/ue_ik_retargeter_maya/core/pole_vector.py')
pole=importlib.util.module_from_spec(spec);spec.loader.exec_module(pole)
fixtures=json.loads((ROOT/'test/fixtures/pbik_cc4_mixamo.json').read_text(encoding='utf-8'))

class PoleVectorTests(unittest.TestCase):
    def test_straight_limb_uses_anatomical_reference(self):
        r=pole.choose_direction([0,0,0],[0,-1,0],[0,-2,0],[0,0,1])
        self.assertEqual(r['source'],'reference')
        self.assertEqual(r['direction'],[0.,0.,1.])
        self.assertTrue(r['orientation_known'])

    def test_tiny_opposite_offsets_do_not_flip_and_are_scale_independent(self):
        for scale in [1e-3,1.,1e3]:
            for noise in [-1e-6,0.,1e-6]:
                r=pole.choose_direction([0,0,0],[0,-scale,noise*scale],[0,-2*scale,0],[0,0,1])
                self.assertGreater(pole.dot(r['direction'],[0,0,1]),.999)

    def test_transition_retains_reference_hemisphere(self):
        for degrees in [2.9,3.1,5.,7.9]:
            offset=-math.tan(math.radians(degrees*.5))
            r=pole.choose_direction([0,0,0],[0,-1,offset],[0,-2,0],[0,0,1])
            self.assertGreater(r['direction'][2],.999)

    def test_clear_animated_bend_is_preserved(self):
        r=pole.choose_direction([0,0,0],[0,-1,-.2],[0,-2,0],[0,0,1])
        self.assertEqual(r['source'],'geometry')
        self.assertLess(r['direction'][2],-.999)
        reference={'local_chord':[0,-1,0],'local_direction':[1,0,0],'orientation_known':False}
        r=pole.resolve_reference(reference,[0,0,0],[0,-1,.2],[0,-2,0],[0,0,0,1])
        self.assertTrue(r['orientation_known'])
        self.assertGreater(r['direction'][2],.999)

    def test_reference_follows_character_rotation(self):
        q=[0,math.sin(math.pi/4),0,math.cos(math.pi/4)]
        ref={'local_chord':[0,-1,0],'local_direction':[0,0,1],'orientation_known':True}
        r=pole.resolve_reference(ref,[0,0,0],[0,-1,0],[0,-2,0],q)
        self.assertGreater(r['direction'][0],.999)

    def test_folded_chord_transport_keeps_reference_plane(self):
        for axis in [[0,1,0],[1,0,0]]:
            r=pole.transport_direction([0,-1,0],[0,0,1],axis)
            self.assertGreater(r[2],.999)
            self.assertAlmostEqual(pole.dot(r,axis),0.)

    def test_straight_positions_alone_do_not_claim_anatomical_knowledge(self):
        r=pole.choose_direction([0,0,0],[0,-1,0],[0,-2,0],[0,0,0])
        self.assertFalse(r['orientation_known'])

    def test_native_fixture_knee_signs_are_calibrated_without_mutation(self):
        for role,f in fixtures.items():
            goal=f['goals'][0]['bone'];knee=f['parents'][goal];hip=f['parents'][knee]
            a,b,c=[f['reference'][i][:3] for i in [hip,knee,goal]]
            toe=next(i for i,parent in enumerate(f['parents']) if parent==goal)
            axis=pole.unit(pole.sub(c,a));front=pole.unit(pole.project(pole.sub(f['reference'][toe][:3],c),axis))
            original=copy.deepcopy(f['bone_settings'][knee]['preferred_angles'])
            r=pole.calibrate_preferred_angles(a,b,c,f['reference'][knee][3:7],original,front)
            self.assertEqual(r['flipped'],role.lower().startswith('target'))
            self.assertGreater(pole.dot(pole.preferred_angle_direction(a,b,c,f['reference'][knee][3:7],r['angles']),front),.9)
            self.assertEqual(original,f['bone_settings'][knee]['preferred_angles'])

    def test_native_crouch_keeps_exactly_straight_and_noisy_knees_forward(self):
        spec=importlib.util.spec_from_file_location('pbik_native',ROOT/'ue58_three_assets_v2/ue_ik_retargeter_maya/core/pbik_native.py')
        native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
        for role,f in fixtures.items():
            for noise in [-1e-5,0.,1e-5]:
                with self.subTest(role=role,noise=noise):
                    reference=copy.deepcopy(f['reference']);bones=copy.deepcopy(f['bone_settings']);knees=[]
                    for goal_index in [0,2]:
                        end=f['goals'][goal_index]['bone'];knee=f['parents'][end];hip=f['parents'][knee]
                        a,b,c=[reference[i][:3] for i in [hip,knee,end]]
                        toe=next(i for i,parent in enumerate(f['parents']) if parent==end)
                        axis=pole.unit(pole.sub(c,a));front=pole.unit(pole.project(pole.sub(reference[toe][:3],c),axis))
                        r=pole.calibrate_preferred_angles(a,b,c,reference[knee][3:7],bones[knee]['preferred_angles'],front)
                        bones[knee]['preferred_angles']=r['angles']
                        upper=pole.length(pole.sub(b,a));lower=pole.length(pole.sub(c,b))
                        reference[knee][:3]=pole.add(pole.add(a,pole.mul(pole.sub(c,a),upper/(upper+lower))),pole.mul(front,noise))
                        knees.append((hip,knee,end,front))
                    poses=copy.deepcopy(reference)
                    for i in range(len(poses)):
                        ancestor=i
                        while ancestor>=0 and ancestor!=f['root']:ancestor=f['parents'][ancestor]
                        if ancestor==f['root']:poses[i][2]-=20.
                    solver=native.NativePBIKSolver(f['parents'],f['root'],reference,[g['bone'] for g in f['goals']])
                    try:out=solver.solve(poses,bones,f['goals'],f['solver_settings'])
                    finally:solver.close()
                    for hip,knee,end,front in knees:
                        result=pole.bend_measure(out[hip][:3],out[knee][:3],out[end][:3])
                        self.assertGreater(pole.dot(result['pole'],front),5.)

class LimbPlaneTests(unittest.TestCase):
    def poses(self,straight=False):
        points=[[0,0,0],[0,-1,0 if straight else .3],[0,-2,0],[0,-2,1],[.1,-.5,.2]]
        return [p+[0.,0.,0.,1.,1.,1.,1.] for p in points]
    def reference(self,poses):
        return pole.build_limb_plane([-1,0,1,2,0],['hip','knee','ankle','ball','twist'],0,1,2,poses)
    def test_swivel_removed_without_moving_endpoints_or_foot_rotation(self):
        inputs=self.poses();solved=copy.deepcopy(inputs);solved[1][:3]=[.3,-1,0];solved[4][:3]=[.2,-.5,-.1]
        out=pole.stabilize_limb_planes(inputs,solved,[self.reference(inputs)])
        self.assertLess(pole.length(pole.sub(out[1][:3],inputs[1][:3])),1e-10)
        self.assertEqual(out[2],solved[2]);self.assertEqual(out[3],solved[3]);self.assertEqual(out[0][:3],solved[0][:3])
        for a,b in [(0,1),(1,2),(0,4)]:self.assertAlmostEqual(pole.length(pole.sub(out[a][:3],out[b][:3])),pole.length(pole.sub(solved[a][:3],solved[b][:3])))
        self.assertEqual(solved[1][:3],[.3,-1,0])
    def test_identity_pose_and_zero_strength(self):
        inputs=self.poses();self.assertEqual(pole.stabilize_limb_planes(inputs,inputs,[self.reference(inputs)]),inputs)
        self.assertIs(pole.stabilize_limb_planes(inputs,inputs,[self.reference(inputs)],0),inputs)
    def test_straight_pose_uses_toe_reference_when_crouched(self):
        inputs=self.poses(True);solved=copy.deepcopy(inputs);solved[2][1]=-1.5;solved[1][:3]=[math.sqrt(1-.75**2),-.75,0]
        out=pole.stabilize_limb_planes(inputs,solved,[self.reference(inputs)])
        self.assertGreater(out[1][2],.6);self.assertAlmostEqual(out[1][0],0.);self.assertEqual(out[2],solved[2])
    def test_animated_reverse_bend_takes_precedence(self):
        reference=self.poses();inputs=self.poses();inputs[1][2]=-.3;solved=copy.deepcopy(inputs);solved[1][:3]=[.3,-1,0]
        out=pole.stabilize_limb_planes(inputs,solved,[self.reference(reference)])
        self.assertLess(out[1][2],-.29)
    def test_partial_strength_and_random_evaluation_order(self):
        inputs=self.poses();solved=copy.deepcopy(inputs);solved[1][:3]=[.3,-1,0];ref=self.reference(inputs)
        half=pole.stabilize_limb_planes(inputs,solved,[ref],.5);self.assertGreater(half[1][0],.2);self.assertGreater(half[1][2],.2)
        full=pole.stabilize_limb_planes(inputs,solved,[ref]);pole.stabilize_limb_planes(self.poses(True),solved,[ref])
        self.assertEqual(full,pole.stabilize_limb_planes(inputs,solved,[ref]))

if __name__=='__main__':unittest.main(verbosity=2)
