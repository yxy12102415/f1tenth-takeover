"""Geometry acceptance for envelope-derived paths; no hardware or ROS."""
import json, math, tempfile, unittest
from pathlib import Path
from test_generic_avoidance import ASSETS, C, S, O, Probe, cluster, pose
ROOT=Path(__file__).resolve().parent
class AdaptiveCorridorTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        a=dict(ASSETS);a['clearance']=[5.]*len(a['clearance'])
        p=Path(self.tmp.name)/'assets.json';p.write_text(json.dumps(a));self.p=Probe(p)
    def tearDown(self):self.p.close();self.tmp.cleanup()
    def ask(self,*args,**kw):
        self.p.close();a=Path(self.tmp.name)/'assets.json';self.p=Probe(a)
        z=self.p.ask(*args,qp_only=True,**kw)
        self.assertTrue(z['valid'],z)
        self.assertEqual(z['score_terms']['planner_geometry'],'obstacle_relative_frenet_corridor')
        return z
    def test_envelope_width_changes_path_without_lattice_quantization(self):
        small=self.ask(clusters=[cluster(1,2.0,w=.38)],cap=.95)
        large=self.ask(clusters=[cluster(1,2.0,w=.50)],cap=.95)
        self.assertGreater(abs(large['offset_m']),abs(small['offset_m'])+.04)
        self.assertGreater(abs(small['offset_m']/.05-round(small['offset_m']/.05)),.02)
    def test_front_car_lateral_position_moves_passing_corridor(self):
        center=self.ask(clusters=[cluster(1,2.0,w=.38)],cap=.95)
        toward_left=self.ask(clusters=[cluster(1,2.0,y=.10,w=.38)],cap=.95)
        self.assertLess(abs(toward_left['offset_m']),abs(center['offset_m'])-.05)
    def test_body_clears_before_merging_not_just_center(self):
        z=self.ask(clusters=[cluster(1,2.0,w=.38)],cap=.95)
        # Independent rectangle SAT via the original explicit prediction API,
        # with present occupied geometry and every 5 cm sampled body pose.
        occupied=z['occupancies']
        for i,p in enumerate(z['path']):
            q=self.p.ask(stamp=1.0,check_prediction=[p],occupancies=occupied)
            self.assertTrue(q['valid'],(i,p,q))
        end=z['path'][-1]
        self.assertLess(math.hypot(end[0]-ASSETS['route'][-1][0],end[1]-ASSETS['route'][-1][1]),1e-6)
        self.assertLess(abs(math.atan2(math.sin(end[2]-ASSETS['route'][-1][2]),math.cos(end[2]-ASSETS['route'][-1][2]))),.01)
    def test_all_corridor_knots_respect_free_intervals(self):
        z=self.ask(clusters=[cluster(1,2.0,w=.38)],cap=.95)
        for s,lo,hi,d in z['score_terms']['corridor_stations']:
            self.assertGreaterEqual(d,lo-1e-7);self.assertLessEqual(d,hi+1e-7)
        self.assertLess(z['maximum_curvature_1pm'],math.tan(.27)/.25+.03)
    def test_infeasible_optimization_seeds_cannot_exceed_motion_offset_limit(self):
        q=json.loads((ROOT/'compact_profile_fixture_20261112.json').read_text())['input']
        z=self.p.ask(**q,cap=.95,max_offset=.55,include_candidates=True)
        self.assertTrue(z['valid'],z)
        accepted=[c for c in z['candidates'] if c['valid']]
        self.assertTrue(accepted)
        self.assertTrue(all(abs(c['offset_m'])<=.550001 for c in accepted))
        self.assertTrue(all(c['score_terms'].get('dense_maximum_curvature_1pm',0)<=math.tan(.27)/.25+.03 for c in accepted))
    def test_measured_motion_is_not_reported_as_stationary_during_latch_delay(self):
        self.p.ask(clusters=[cluster(1,2.0)])
        records=[]
        for i in range(1,6):
            z=self.p.ask(stamp=1+i*.2,clusters=[cluster(1,2.0+i*.08)])
            records.extend(z['occupancies'])
        moving=[o for o in records if math.hypot(o['vx'],o['vy'])>.08]
        self.assertTrue(moving)
        self.assertTrue(all(not o['stationary_history'] for o in moving))
    def test_no_feasible_corridor_is_not_forced_through_a_wall(self):
        z=self.p.ask(clusters=[cluster(1,1.8,0,.3),cluster(2,1.8,-.6,.3),cluster(3,1.8,.6,.3)],max_offset=.55,cap=.95)
        self.assertFalse(z['valid']);self.assertEqual(z['reason'],'generic_no_feasible_path')
if __name__=='__main__':unittest.main(verbosity=2)
