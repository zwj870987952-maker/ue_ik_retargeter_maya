"""Pure-model invariants: timing, percentages, independent effects and quaternions."""
import importlib.util
import math
from pathlib import Path
import unittest

PATH=Path(__file__).resolve().parents[1]/'ue58_three_assets_v2/ue_ik_retargeter_maya/animation_workbench/engine.py'
spec=importlib.util.spec_from_file_location('aw_engine',PATH);engine=importlib.util.module_from_spec(spec);spec.loader.exec_module(engine)

def source(scale=1.):
    return dict(start=0,positions=[[scale*t,scale*math.sin(t*.13),0.] for t in range(41)],
                rotations=[[0.,math.sin(math.radians(t*3.)/2),0.,math.cos(math.radians(t*3.)/2)] for t in range(41)])

def marker(**overrides):
    settings=dict(engine.DEFAULTS);settings.update(overrides)
    return dict(frame=20,settings=settings)

class RhythmModelTests(unittest.TestCase):
    def test_zero_hold_and_effects_is_identity(self):
        src=source();r=engine.evaluate(src,[marker(hold_left=0,hold_right=0,settle_strength=0,prep_strength=0)],1,40)
        self.assertEqual(r['positions'],src['positions']);self.assertTrue(all(abs(a-b)<1e-9 for a,b in zip(r['source_times'],r['times'])))
    def test_asymmetric_hold_and_fixed_boundaries(self):
        src=source();r=engine.evaluate(src,[marker(hold_left=2,hold_right=4)],1,40)
        for frame in range(18,25):
            self.assertLess(engine.length(engine.sub(r['positions'][frame],src['positions'][20])),1e-9)
            self.assertLess(engine.quaternion_angle(r['rotations'][frame],src['rotations'][20]),1e-5)
        for frame in [0,1,20,40]:self.assertLess(engine.length(engine.sub(r['positions'][frame],src['positions'][frame])),1e-9)
        self.assertTrue(all(isinstance(t,int) for t in r['times']))
    def test_each_side_can_be_zero(self):
        for left,right in [(0,4),(4,0),(0,0)]:
            r=engine.evaluate(source(),[marker(hold_left=left,hold_right=right)],1,40)
            self.assertEqual(r['events'][0]['hold_start'],20-left);self.assertEqual(r['events'][0]['hold_end'],20+right)
    def test_time_map_monotone(self):
        r=engine.evaluate(source(),[marker()],1,40)
        self.assertTrue(all(b>=a-1e-9 for a,b in zip(r['source_times'],r['source_times'][1:])))
    def test_independent_hold_strength(self):
        src=source();r=engine.evaluate(src,[marker(hold_strength=0,settle_strength=0,prep_strength=0)],1,40)
        self.assertEqual(r['positions'],src['positions'])
    def test_effects_never_modify_hold(self):
        src=source();plain=engine.evaluate(src,[marker(settle_strength=0,prep_strength=0)],1,40)
        enhanced=engine.evaluate(src,[marker(settle_amplitude=30,prep_amplitude=30)],1,40)
        for i in range(18,24):self.assertEqual(plain['positions'][i],enhanced['positions'][i])
    def test_independent_settle_and_prep(self):
        plain=engine.evaluate(source(),[marker(settle_strength=0,prep_strength=0)],1,40)
        arrival=engine.evaluate(source(),[marker(prep_strength=0)],1,40)
        departure=engine.evaluate(source(),[marker(settle_strength=0)],1,40)
        for t in range(20,41):self.assertEqual(plain['positions'][t],arrival['positions'][t])
        for t in range(21):self.assertEqual(plain['positions'][t],departure['positions'][t])
    def test_percentage_spatial_scale(self):
        a=engine.evaluate(source(),[marker()],1,40);b=engine.evaluate(source(3.),[marker()],1,40)
        for p,q in zip(a['positions'],b['positions']):self.assertLess(engine.length(engine.sub(engine.mul(p,3.),q)),1e-8)
    def test_no_markers_restores_source(self):
        src=source();self.assertEqual(engine.evaluate(src,[],1,40)['positions'],src['positions'])
    def test_neighbour_and_end_markers(self):
        r=engine.evaluate(source(),[dict(frame=i,settings=engine.DEFAULTS) for i in [1,19,20,40]],1,40)
        self.assertTrue(all(math.isfinite(v) for p in r['positions'] for v in p))
    def test_shortest_quaternion_path(self):
        a=[0.,math.sin(math.radians(170)/2),0.,math.cos(math.radians(170)/2)]
        b=[0.,math.sin(math.radians(-170)/2),0.,math.cos(math.radians(-170)/2)]
        self.assertLess(engine.quaternion_angle(engine.slerp(a,b,.5),[0,1,0,0]),1e-5)
    def test_contact_floor_envelope(self):
        src=source();src['positions']=[[p[0],max(0.,p[1]),p[2]] for p in src['positions']]
        r=engine.evaluate(src,[marker()],1,40,foot=True)
        self.assertTrue(all(p[1]>=-1e-9 for p in r['positions']))

if __name__=='__main__':unittest.main()
