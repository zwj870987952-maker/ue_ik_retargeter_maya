"""Physical regression tests for the UE 5.8 native PBIK adapter, without Maya."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('pbik_native',ROOT/'ue58_three_assets_v2/ue_ik_retargeter_maya/core/pbik_native.py')
native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
FIXTURES=json.loads((ROOT/'test/fixtures/pbik_cc4_mixamo.json').read_text(encoding='utf-8'))

def rotation_error(a,b):
    dot=sum(x*y for x,y in zip(a,b))/math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    return math.degrees(2*math.acos(min(1.,abs(dot))))

class PBIKTests(unittest.TestCase):
    def solve(self,f,goals=None,bones=None,settings=None):
        solver=native.NativePBIKSolver(f['parents'],f['root'],f['reference'],[g['bone'] for g in f['goals']])
        try:return solver.solve(f['reference'],bones or f['bone_settings'],goals or f['goals'],settings or f['solver_settings'])
        finally:solver.close()

    def test_coordinate_and_bone_frame_round_trip(self):
        for f in FIXTURES.values():
            for pose in f['reference']:
                p,q,s=native.from_engine_pose(pose);back=native.to_engine_pose(p,q,s)
                self.assertLess(max(abs(a-b) for a,b in zip(back,pose)),1.e-12)

    def test_zero_offset_preserves_input_pose(self):
        for role,f in FIXTURES.items():
            with self.subTest(role=role):
                out=self.solve(f)
                self.assertLess(max(math.dist(p[:3],q[:3]) for p,q in zip(out,f['reference'])),1.e-6)

    def test_all_goals_remain_constrained_when_one_hand_moves(self):
        for role,f in FIXTURES.items():
            with self.subTest(role=role):
                goals=copy.deepcopy(f['goals']);goals[1]['position'][1]+=20
                out=self.solve(f,goals)
                # A finite 20/10 iteration solve has residual error. The old
                # untouched-arm shortcut drifted 16-17cm in this fixture.
                self.assertLess(math.dist(out[goals[3]['bone']][:3],goals[3]['position']),3.)
                for g in goals:self.assertLess(rotation_error(out[g['bone']][3:7],g['rotation']),1.e-5)

    def test_rotation_alpha_is_applied_once_and_independent_of_position(self):
        for role,f in FIXTURES.items():
            for index in [0,1]:
                with self.subTest(role=role,goal=index):
                    goals=copy.deepcopy(f['goals']);g=goals[index];original=g['rotation']
                    g['rotation']=native.multiply([math.sin(math.pi/6),0,0,math.cos(math.pi/6)],original)
                    g['rotation_alpha']=.5;g['position_alpha']=0.
                    out=self.solve(f,goals)
                    self.assertAlmostEqual(rotation_error(out[g['bone']][3:7],original),30.,places=5)

    def test_bone_stiffness_changes_the_pose(self):
        for role,f in FIXTURES.items():
            with self.subTest(role=role):
                goals=copy.deepcopy(f['goals']);goals[1]['position'][1]+=60
                loose=copy.deepcopy(f['bone_settings']);locked=copy.deepcopy(f['bone_settings'])
                for b in loose:b['rotation_stiffness']=0.
                for b in locked:b['rotation_stiffness']=1.
                a=self.solve(f,goals,loose);b=self.solve(f,goals,locked)
                self.assertGreater(max(math.dist(p[:3],q[:3]) for p,q in zip(a,b)),1.)

    def test_pin_root_uses_input_transform(self):
        for role,f in FIXTURES.items():
            with self.subTest(role=role):
                goals=copy.deepcopy(f['goals']);goals[1]['position'][1]+=60
                settings=dict(f['solver_settings'],root_behavior='PinToInput')
                out=self.solve(f,goals,settings=settings);r=f['root']
                self.assertLess(math.dist(out[r][:3],f['reference'][r][:3]),1.e-6)
                self.assertLess(rotation_error(out[r][3:7],f['reference'][r][3:7]),1.e-5)

    def test_strength_zero_disables_position_pull(self):
        for role,f in FIXTURES.items():
            with self.subTest(role=role):
                goals=copy.deepcopy(f['goals']);goals[1]['position'][1]+=60;goals[1]['strength_alpha']=0.
                out=self.solve(f,goals)
                self.assertLess(math.dist(out[goals[1]['bone']][:3],f['reference'][goals[1]['bone']][:3]),1.e-5)

if __name__=='__main__':unittest.main(verbosity=2)
