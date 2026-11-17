"""Native planner/occupancy tests. No ROS, drivers or actuator topics."""
import json, math, subprocess, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent
ASSETS=json.loads((ROOT/'runtime_assets.json').read_text())
O=ASSETS['route'][0]; C,S=math.cos(O[2]),math.sin(O[2])
def pose(x=0,y=0,h=0):return [O[0]+C*x-S*y,O[1]+S*x+C*y,O[2]+h]
def cluster(i,x,y=0,w=.30,l=.30):
    return dict(id=i,hits=6,points=[pose(x+dx,y+dy)[:2] for dx,dy in [(-l/2,-w/2),(-l/2,w/2),(l/2,-w/2),(l/2,w/2)]])
class Probe:
    def __init__(self,assets=None):
        self.p=subprocess.Popen([str(ROOT/'bin/generic_probe'),str(assets or ROOT/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def ask(self,stamp=1,clusters=(),**kw):
        payload=dict(stamp=stamp,poses=[pose()],clusters=list(clusters));payload.update(kw)
        self.p.stdin.write(json.dumps(payload)+'\n');self.p.stdin.flush()
        line=self.p.stdout.readline()
        if not line:raise RuntimeError('probe exited')
        return json.loads(line)
    def close(self):self.p.stdin.close();self.p.wait(timeout=5);self.p.stdout.close()
class GenericTest(unittest.TestCase):
    def setUp(self):self.p=Probe()
    def tearDown(self):self.p.close()
    def test_stationary_velocity_noise_does_not_integrate_into_body_extent(self):
        # Rolling visible surface samples remain inside one fixed body. Their
        # nearest-neighbour phase can suggest a small persistent velocity even
        # though the physical support never leaves the measured union.
        for k in range(120):
            points=[pose(1.65+.02*j+.002*(k%10),y)[:2]
                    for j in range(15) for y in [-.15,.15]]
            r=self.p.ask(stamp=1+k*.2,clusters=[dict(id=1,hits=6,points=points)])
        o=r['occupancies'][0]
        self.assertFalse(o['has_moved'])
        self.assertLessEqual(o['xmax']-o['xmin'],.298*abs(C)+.30*abs(S)+1e-6)
        self.assertLessEqual(o['ymax']-o['ymin'],.298*abs(S)+.30*abs(C)+1e-6)
    def test_safe_held_path_survives_loss_of_preferred_margin(self):
        first=self.p.ask(clusters=[cluster(1,1.8)])
        self.assertTrue(first['valid'])
        # Updated from .35 to .36: the acceleration-aware arrival model
        # now correctly rejects .35 as a future hard-gap conflict.
        second=self.p.ask(stamp=1.2,clusters=[cluster(1,1.8),cluster(2,0,.36,.02,.02)])
        self.assertTrue(second['valid'],second)
        self.assertEqual(first['path'],second['path'])
        self.assertTrue(second['score_terms']['generic_held_path'])
        self.assertAlmostEqual(second['score_terms']['generic_required_body_gap_m'],.08)
        third=self.p.ask(stamp=1.4,clusters=[cluster(1,1.8),cluster(2,0,.33,.02,.02)])
        # Unsafe held geometry must be replaced or rejected. A corridor solver
        # may fail closed in this tight close-start rear-swing configuration.
        if third['valid']:self.assertNotEqual(second['path'],third['path'])
        else:
            self.assertEqual(third['path'],[])
            self.assertEqual(third['reason'],'generic_no_feasible_path')
    def test_isolated_near_chassis_raw_return_is_not_a_confirmed_cluster(self):
        inc=4.71238898/1080;amin=-2.35619449
        k=round((math.atan2(.158,.164-.27)-amin)/inc)
        ranges=[None]*1081;ranges[k]=math.hypot(.164-.27,.158)
        r=self.p.ask(ranges=ranges,angle_min=amin,angle_increment=inc,range_min=.05,range_max=10.)
        self.assertEqual(r['perception']['raw_scan_point_count'],1)
        self.assertEqual(r['perception']['ego_body_returns_removed'],0)
        self.assertEqual(r['occupancies'],[])
        self.assertTrue(r['valid'],r)
    def test_corroborated_near_chassis_raw_obstacle_still_blocks(self):
        inc=4.71238898/1080;amin=-2.35619449
        k=round((math.atan2(.158,.164-.27)-amin)/inc)
        ranges=[None]*1081
        for j in [k-1,k,k+1]:ranges[j]=math.hypot(.164-.27,.158)
        r=self.p.ask(ranges=ranges,angle_min=amin,angle_increment=inc,range_min=.05,range_max=10.)
        self.assertEqual(r['perception']['raw_scan_point_count'],3)
        self.assertGreater(len(r['occupancies']),0)
        self.assertFalse(r['valid'])
    def test_refresh_only_validates_existing_path_with_fresh_observations(self):
        a=self.p.ask(clusters=[cluster(1,1.8)])
        b=self.p.ask(stamp=1.2,clusters=[cluster(1,1.8)],allow_search=False)
        self.assertTrue(b['valid'],b)
        self.assertEqual(a['path'],b['path'])
        self.assertTrue(b['score_terms']['generic_held_path'])
        self.assertEqual(b['occupancies'][0]['stamp'],1.2)
    def test_refresh_only_rejects_new_obstacle_without_stale_fallback(self):
        a=self.p.ask(clusters=[cluster(1,1.8)])
        q=a['path'][30]
        obstacle=dict(id=2,hits=6,points=[[q[0]+dx,q[1]+dy] for dx,dy in [(-.1,-.1),(-.1,.1),(.1,-.1),(.1,.1)]])
        b=self.p.ask(stamp=1.2,clusters=[cluster(1,1.8),obstacle],allow_search=False)
        self.assertFalse(b['valid'])
        self.assertEqual(b['path'],[])
    def test_compact_profile_fits_recorded_prestart_occupancy(self):
        q=json.loads((ROOT/'compact_profile_fixture_20261112.json').read_text())['input']
        z=self.p.ask(**q,cap=.95,max_offset=.55)
        self.assertTrue(z['valid'],z)
        self.assertLessEqual(abs(z['offset_m']),.55)
        self.assertLess(z['maximum_curvature_1pm'],.8)
    def test_emergency_straight_coast_covers_delay_and_full_stop(self):
        z=self.p.ask(coast_map=True,speed=.8,servo=0.)
        self.assertTrue(z['valid'],z)
        start,end=z['path'][0],z['path'][-1]
        distance=math.hypot(end[0]-start[0],end[1]-start[1])
        self.assertAlmostEqual(distance,.8*.1+.8*.8/(2*.4),places=5)
    def test_emergency_coast_rejects_wall_before_overlap(self):
        q=[1.24,-.665,-.61]
        z=self.p.ask(coast_map=True,poses=[q],speed=0.,servo=0.)
        self.assertTrue(z['valid'],z)
        z=self.p.ask(coast_map=True,poses=[q],speed=1.25,servo=0.)
        self.assertFalse(z['valid'])
        self.assertEqual(z['reason'],'generic_emergency_coast_map_conflict')
    def test_return_does_not_double_count_vehicle_length(self):
        q=json.loads((ROOT/'compact_profile_fixture_20261112.json').read_text())['input']
        z=self.p.ask(**q,cap=.95,max_offset=.55)
        self.assertTrue(z['valid'],z)
        peak=abs(z['offset_m']);reached=False;return_station=None
        for x,y,_ in z['path']:
            station=C*(x-O[0])+S*(y-O[1]);offset=-S*(x-O[0])+C*(y-O[1])
            if abs(offset)>peak-.003:reached=True
            if reached and abs(offset)<peak-.015:
                return_station=station;break
        self.assertIsNotNone(return_station)
        self.assertLess(return_station,3.1)
    def test_no_target_required(self):
        r=self.p.ask();self.assertTrue(r['valid'],r);self.assertEqual(r['offset_m'],0)
    def test_static_vehicle_bypass(self):
        r=self.p.ask(clusters=[cluster(1,1.8)]);self.assertTrue(r['valid'],r);self.assertGreater(abs(r['offset_m']),.4)
    def test_recorded_early_stop_occupancy_snapshot(self):
        payload=json.loads((ROOT/'regression_20261111_early_stop.json').read_text())
        r=self.p.ask(**payload)
        # This recording contains unresolved close-start returns. A local
        # planning fix must not turn those measured conflicts into free space.
        self.assertFalse(r['valid'])
        self.assertEqual(r['reason'],'generic_no_feasible_path')
        self.assertTrue(set(r['candidate_reasons']) & {'generic_predicted_occupancy_conflict','generic_corridor_infeasible'})
    def test_preferred_gap_falls_back_to_existing_hard_gap(self):
        r=self.p.ask(clusters=[cluster(1,0,.33,.02,.02)])
        self.assertTrue(r['valid'],r)
        self.assertAlmostEqual(r['score_terms']['generic_required_body_gap_m'],.08)
    def test_roadside_region_does_not_postpone_local_return(self):
        r=self.p.ask(clusters=[cluster(1,1.8),cluster(2,4.6,.9,.1,.1)])
        self.assertTrue(r['valid'],r)
        self.assertGreater(abs(r['offset_m']),.4)
    def test_distant_occupancy_is_replanned_before_approach(self):
        r=self.p.ask(clusters=[cluster(1,1.8),cluster(2,5.8,0,.8,.5)])
        self.assertTrue(r['valid'],r)
        q=self.p.ask(stamp=1.05,check_prediction=[pose(5.8)],occupancies=r['occupancies'])
        self.assertFalse(q['valid'])
        self.assertIn('occupancy_conflict',q['reason'])
    def test_splitting_wheels_does_not_require_identity(self):
        r=self.p.ask(clusters=[cluster(i+1,1.8+sx*.10,sy*.12,.06,.08) for i,(sx,sy) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)])]);self.assertTrue(r['valid'],r)
    def test_unmatched_target_retained_without_target_lost(self):
        self.p.ask(clusters=[cluster(1,1.8)])
        r=self.p.ask(stamp=1.4);self.assertEqual(len(r['occupancies']),1);self.assertEqual(r['occupancies'][0]['stamp'],1)
        self.assertNotIn('target_lost',r['reason'])
    def test_clear_requires_positive_evidence(self):
        self.p.ask(clusters=[cluster(1,1.8)])
        for t in [1.05,1.10]:
            r=self.p.ask(stamp=t,free_range=6);self.assertEqual(len(r['occupancies']),1)
        r=self.p.ask(stamp=1.15,free_range=6);self.assertEqual(r['occupancies'],[])
    def test_new_identity_does_not_throw_ambiguity(self):
        self.p.ask(clusters=[cluster(1,1.8)])
        r=self.p.ask(stamp=1.05,clusters=[cluster(10,1.8)])
        self.assertTrue(r['valid'],r);self.assertEqual(len(r['occupancies']),1)
        self.assertEqual(r['occupancies'][0]['stamp'],1.05)
    def test_side_obstacle_not_assigned_to_lead(self):
        r=self.p.ask(clusters=[cluster(42,2.5,.75,.10,.10)]);self.assertTrue(r['valid'],r);self.assertEqual(r['offset_m'],0)
    def test_full_body_conflict_rejected(self):
        r=self.p.ask(clusters=[cluster(42,.4)]);self.assertFalse(r['valid'])
    def test_stale_moving_occupancy_expands_not_erased(self):
        for i in range(10):self.p.ask(stamp=1+i*.05,clusters=[cluster(42,1.8+i*.015)])
        r=self.p.ask(stamp=5);self.assertEqual(len(r['occupancies']),1)
        self.assertTrue(r['occupancies'][0]['has_moved']);self.assertFalse(r['valid'])
    def test_controller_prediction_checks_all_objects(self):
        r=self.p.ask(clusters=[cluster(2,1.8),cluster(3,3.8,.3)])
        q=self.p.ask(stamp=1.01,check_prediction=[pose(1.7)],occupancies=r['occupancies']);self.assertFalse(q['valid']);self.assertIn('occupancy_conflict',q['reason'])
    def test_forged_bounds_rejected(self):
        r=self.p.ask(stamp=1,check_prediction=[pose()],occupancies=[dict(id=1,stamp=1,xmin=2,xmax=1,ymin=0,ymax=1,vx=0,vy=0,sensor_x=0,sensor_y=0)]);self.assertFalse(r['valid'])
    def test_timestamp_replay_rejected(self):
        self.p.ask();r=self.p.ask();self.assertFalse(r['valid']);self.assertEqual(r['reason'],'occupancy_scan_stamp')
    def test_velocity_estimation_uses_measurement_timestamps(self):
        for i in range(6):r=self.p.ask(stamp=1+i*.05,clusters=[cluster(1,1.8+i*.01)])
        v=r['occupancies'][0]
        self.assertGreater(v['vx']*C+v['vy']*S,.12)
        self.assertLess(v['vx']*C+v['vy']*S,.25)
    def test_valid_incumbent_is_not_reanchored(self):
        first=self.p.ask(clusters=[cluster(1,1.8)])
        r=self.p.ask(stamp=1.05,clusters=[cluster(1,1.8)])
        self.assertEqual(first['path'],r['path'])
    def test_both_sides_blocked_rejects(self):
        r=self.p.ask(clusters=[cluster(1,1.8,0,.3),cluster(2,1.8,-.6,.3),cluster(3,1.8,.6,.3)])
        self.assertFalse(r['valid'])
    def test_left_candidate_when_right_is_occupied(self):
        import tempfile
        self.p.close()
        a=dict(ASSETS);a['clearance']=[5.]*len(a['clearance'])
        with tempfile.TemporaryDirectory() as td:
            filename=Path(td)/'assets.json';filename.write_text(json.dumps(a))
            self.p=Probe(filename)
            r=self.p.ask(clusters=[cluster(1,1.8),cluster(2,2.8,-.75,.3)])
            self.assertTrue(r['valid'],r)
            self.assertGreater(r['offset_m'],0)
    def test_static_surface_change_is_not_centroid_motion(self):
        rear=[pose(1.65,-.15+j*.01)[:2] for j in range(31)]
        front=dict(id=1,hits=6,points=rear)
        self.p.ask(clusters=[front])
        front['points']=rear+[pose(1.65+j*.02,-.15)[:2] for j in range(1,11)]
        r=self.p.ask(stamp=1.1,clusters=[front]);o=r['occupancies'][0]
        self.assertLess(abs(o['vx']),.03);self.assertLess(abs(o['vy']),.03)
    def test_normal_source_age_does_not_instantly_assume_full_speed(self):
        occupied=[dict(id=1,hits=21,stamp=1.,xmin=1.6422,xmax=1.6477,ymin=-.1052,ymax=.1858,vx=0.,vy=0.,sensor_x=0.,sensor_y=0.)]
        # This observed near-rest scan used to be expanded at .55m/s for
        # ordinary 0.25s planner transport age, before the predicted horizon.
        r=self.p.ask(stamp=1.27,check_prediction=[[.874,-.238,-.483]]*11,occupancies=occupied)
        self.assertTrue(r['valid'],r)
    def test_different_motion_cannot_be_averaged_into_one_static_box(self):
        for i in range(7):
            r=self.p.ask(stamp=1+i*.05,clusters=[cluster(1,1.8+i*.01,-.10,.10,.10),cluster(2,1.8-i*.01,.10,.10,.10)])
        self.assertEqual(len(r['occupancies']),2)
        vx=[o['vx']*C+o['vy']*S for o in r['occupancies']]
        self.assertLess(min(vx),-.1);self.assertGreater(max(vx),.1)
    def test_short_lived_fragment_inside_current_measured_body_is_absorbed(self):
        fragment=dict(id=3,hits=1,points=[pose(1.9,0)[:2]])
        self.p.ask(clusters=[cluster(1,1.8),fragment])
        r=self.p.ask(stamp=1.2,clusters=[cluster(1,1.8)])
        self.assertEqual(len(r['occupancies']),1)
        self.assertEqual(r['occupancies'][0]['stamp'],1.2)
    def test_uncovered_low_hit_obstacle_is_not_absorbed(self):
        fragment=dict(id=3,hits=1,points=[pose(2.5,.5)[:2]])
        self.p.ask(clusters=[cluster(1,1.8),fragment])
        r=self.p.ask(stamp=1.2,clusters=[cluster(1,1.8)])
        self.assertEqual(len(r['occupancies']),2)
        self.assertTrue(any(o['stamp']==1 for o in r['occupancies']))
    def test_map_guard_checks_front_overhang_and_heading(self):
        clear=self.p.ask(stamp=1,check_prediction=[[2.10,-.79,0]],occupancies=[])
        rotated=self.p.ask(stamp=1,check_prediction=[[2.10,-.79,math.pi/2]],occupancies=[])
        self.assertTrue(clear['valid'],clear)
        self.assertFalse(rotated['valid']);self.assertEqual(rotated['reason'],'generic_full_body_map')
    def test_low_hit_hidden_surface_uses_current_mature_body_envelope(self):
        fragment=dict(id=3,hits=1,points=[pose(1.9,0)[:2]])
        rear=dict(id=1,hits=8,points=[pose(1.65,-.15+j*.01)[:2] for j in range(31)])
        self.p.ask(clusters=[rear,fragment])
        r=self.p.ask(stamp=1.2,clusters=[rear])
        self.assertEqual(len(r['occupancies']),1)
    def test_established_wheel_covered_by_fresh_body_is_associated_geometrically(self):
        self.p.ask(clusters=[cluster(2,1.70,.14,.06,.08)])
        r=self.p.ask(stamp=1.2,clusters=[cluster(10,1.8)])
        self.assertEqual(len(r['occupancies']),1)
        self.assertEqual(r['occupancies'][0]['stamp'],1.2)
    def test_uncovered_established_obstacle_is_preserved(self):
        self.p.ask(clusters=[cluster(2,2.5,.5,.06,.08)])
        r=self.p.ask(stamp=1.2,clusters=[cluster(10,1.8)])
        self.assertEqual(len(r['occupancies']),2)
        self.assertTrue(any(o['stamp']==1 for o in r['occupancies']))
    def test_partial_view_keeps_learned_body_extent(self):
        first=self.p.ask(clusters=[cluster(1,1.8)])
        partial=dict(id=1,hits=7,points=[pose(1.94,.14)[:2]])
        r=self.p.ask(stamp=1.2,clusters=[partial]);o=r['occupancies'][0]
        self.assertLessEqual(o['ymin'],first['occupancies'][0]['ymin']+.002)
        self.assertGreaterEqual(o['xmax']-o['xmin'],.29)
    def test_real_new_raw_id_with_one_hit_continues_geometric_component(self):
        self.p.ask(clusters=[cluster(1,1.8)])
        partial=dict(id=99,hits=1,points=[pose(1.94,.14)[:2]])
        r=self.p.ask(stamp=1.2,clusters=[partial])
        self.assertEqual(len(r['occupancies']),1)
        self.assertEqual(r['occupancies'][0]['stamp'],1.2)
        self.assertGreaterEqual(r['occupancies'][0]['hits'],6)
    def test_tentative_missing_noise_has_bounded_memory(self):
        one=dict(id=1,hits=1,points=[pose(2.5,.5)[:2]])
        self.p.ask(clusters=[one]);r=self.p.ask(stamp=1.49)
        self.assertEqual(len(r['occupancies']),1)
        r=self.p.ask(stamp=1.51);self.assertEqual(r['occupancies'],[])
    def test_confirmed_missing_obstacle_does_not_expire_as_noise(self):
        self.p.ask(clusters=[cluster(1,2.5,.5)])
        r=self.p.ask(stamp=2.);self.assertEqual(len(r['occupancies']),1)
        self.assertEqual(r['occupancies'][0]['stamp'],1.)
    def test_self_filter_stays_strictly_inside_declared_physical_body(self):
        r=self.p.ask(ego_mask_points=[[.183,.140],[.18,.15],[.25,0],[-.27,0],[0,.155],[.30,.14],[0,.165]])
        self.assertEqual(r['masked'],[True,True,False,False,False,False,False])
    def test_stationary_confirmed_region_does_not_expand_across_whole_room(self):
        c=cluster(1,1.8);c['hits']=12
        self.p.ask(clusters=[c]);r=self.p.ask(stamp=31)
        self.assertTrue(r['valid'],r)
        self.assertTrue(r['occupancies'][0]['stationary_history'])
        self.assertEqual(r['occupancies'][0]['stamp'],1.)
    def test_observed_motion_is_retained_through_a_later_miss(self):
        for i in range(10):
            c=cluster(1,1.8+i*.015);c['hits']=8+i
            r=self.p.ask(stamp=1+i*.05,clusters=[c])
        self.assertTrue(r['occupancies'][0]['has_moved'])
        lost=self.p.ask(stamp=1.7)
        self.assertTrue(lost['occupancies'][0]['has_moved'])
        self.assertFalse(lost['occupancies'][0]['stationary_history'])
    def test_stationary_component_reassociation_is_not_tied_to_raw_id_or_age(self):
        self.p.ask(clusters=[cluster(1,2.5,.5)])
        r=self.p.ask(stamp=5,clusters=[dict(id=99,hits=1,points=cluster(1,2.5,.5)['points'])])
        self.assertEqual(len(r['occupancies']),1)
        self.assertEqual(r['occupancies'][0]['stamp'],5)
        self.assertGreaterEqual(r['occupancies'][0]['hits'],7)
if __name__=='__main__':unittest.main(verbosity=2)
