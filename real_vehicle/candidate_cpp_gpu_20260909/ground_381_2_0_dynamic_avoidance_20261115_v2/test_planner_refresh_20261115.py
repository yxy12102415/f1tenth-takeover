"""Non-actuating regressions for planner timing reuse and search failures."""
import unittest,json
from test_generic_avoidance import Probe,ASSETS,ROOT,pose
class PlannerRefresh(unittest.TestCase):
 def setUp(self):self.p=Probe()
 def tearDown(self):self.p.close()
 def test_prediction_horizon_preserves_every_relevant_arrival(self):
  path=[pose(i*.05) for i in range(101)]
  full=self.p.ask(arrival_path=path,speed=0.,cap=2.15)
  short=self.p.ask(arrival_path=path,speed=0.,cap=2.15,arrival_horizon=1.2)
  self.assertTrue(full['valid']);self.assertTrue(short['valid'])
  crossed=False
  for a,b in zip(full['times'],short['times']):
   if not crossed:self.assertAlmostEqual(a,b,places=12)
   else:self.assertGreater(b,1.2)
   if a>1.2:crossed=True
  self.assertTrue(crossed)
 def test_trimmed_geometry_cache_matches_fresh_computation(self):
  path=ASSETS['route'][:60]
  warm=self.p.ask(map_limit_path=path)
  self.assertTrue(warm['valid'],warm)
  trim=path[2:]
  held=self.p.ask(map_limit_path=trim,poses=[trim[0]])
  fresh=Probe()
  try:reference=fresh.ask(map_limit_path=trim,poses=[trim[0]])
  finally:fresh.close()
  self.assertTrue(held['valid'],held);self.assertTrue(reference['valid'],reference)
  self.assertEqual(held['limits'],reference['limits'])
  self.assertEqual(warm['limits'][2:],held['limits'])
 def test_infeasible_timing_candidate_does_not_abort_other_candidates(self):
  fixture=json.loads((ROOT/'planner_refresh_fixture_20261115.json').read_text());front=fixture['occupancies']
  z=self.p.ask(poses=[ASSETS['route'][0]],stamp=fixture['stamp'],plan_occupancies=front,
    speed=0.,cap=2.15,max_offset=.58,seed_reserve=.07,map_speed_feasibility=True,search_budget_ms=1000,include_candidates=True)
  self.assertIn(z['reason'],('generic_validated','generic_no_feasible_path'),z)
  self.assertTrue(z.get('candidates'),z)
  self.assertTrue(any(v['reason']=='map_coast_below_drivable_floor' for v in z['candidates']),z.get('candidate_reasons'))
if __name__=='__main__':unittest.main(verbosity=2)
