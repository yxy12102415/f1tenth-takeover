"""Explicit stationary-scene commissioning; moving mode remains independent."""
import json,unittest
from test_generic_avoidance import ROOT,Probe,cluster,pose
class StationaryMemory(unittest.TestCase):
 def setUp(self):self.p=Probe()
 def tearDown(self):self.p.close()
 def test_recorded_old_motion_does_not_expand_static_scene_across_route(self):
  f=json.loads((ROOT/'stationary_memory_fixture_20261115.json').read_text())
  z=self.p.ask(poses=[f['pose']],stamp=f['stamp'],plan_occupancies=f['occupancies'],stationary_scene=True,
    speed=0.,cap=2.15,max_offset=.58,seed_reserve=.07,map_speed_feasibility=True,search_budget_ms=1000)
  self.assertTrue(z['valid'],z)
  self.assertGreater(len(f['occupancies']),50)
 def test_static_occluded_body_is_retained_and_still_blocks_collision(self):
  first=self.p.ask(stationary_scene=True,clusters=[cluster(1,1.8)])
  self.assertTrue(first['valid'],first)
  late=self.p.ask(stationary_scene=True,stamp=8,clusters=[])
  self.assertTrue(late['occupancies'],late)
  for o in late['occupancies']:
   self.assertEqual(o['vx'],0.);self.assertEqual(o['vy'],0.)
  collision=self.p.ask(stationary_scene=True,stamp=8,check_prediction=[pose(1.8)],occupancies=late['occupancies'])
  self.assertFalse(collision['valid'],collision)
  self.assertEqual(collision['reason'],'generic_predicted_occupancy_conflict')
 def test_fixed_obstacle_does_not_accelerate_into_clear_future_corridor(self):
  observed=self.p.ask(stationary_scene=True,clusters=[cluster(1,1.8)])
  objects=observed['occupancies']
  self.assertTrue(all(o['fixed_world'] for o in objects))
  z=self.p.ask(stamp=1.,check_prediction=[pose(1.8,.46)]*25,occupancies=objects)
  self.assertTrue(z['valid'],z)
 def test_moving_scene_keeps_future_motion_uncertainty(self):
  observed=self.p.ask(clusters=[cluster(1,1.8)])
  objects=observed['occupancies']
  self.assertTrue(all(not o['fixed_world'] for o in objects))
  z=self.p.ask(stamp=1.,check_prediction=[pose(1.8,.46)]*25,occupancies=objects)
  self.assertFalse(z['valid'],z)
  self.assertEqual(z['reason'],'generic_predicted_occupancy_conflict')
if __name__=='__main__':unittest.main(verbosity=2)
