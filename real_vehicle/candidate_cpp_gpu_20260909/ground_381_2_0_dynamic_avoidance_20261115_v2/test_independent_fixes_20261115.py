"""Non-actuating regressions for timing, moving-shape transport and search leases."""
import unittest,math,json
from test_generic_avoidance import Probe,pose,cluster,ROOT
class IndependentFixes(unittest.TestCase):
 def setUp(self):self.p=Probe()
 def tearDown(self):self.p.close()
 def test_rest_arrival_includes_acceleration_not_distance_over_speed_cap(self):
  path=[pose(i*.1) for i in range(21)]
  z=self.p.ask(arrival_path=path,speed=0.,cap=2.15)
  self.assertTrue(z['valid'],z);self.assertGreater(z['times'][-1],1.35)
  self.assertGreater(z['times'][1],.30)
  rolling=self.p.ask(arrival_path=path,speed=2.,cap=2.15)
  self.assertAlmostEqual(rolling['times'][-1],1.,places=5)
 def test_slower_arrival_rechecks_old_close_side_fixture(self):
  first=self.p.ask(clusters=[cluster(1,1.8)])
  second=self.p.ask(stamp=1.2,clusters=[cluster(1,1.8),cluster(2,0,.35,.02,.02)])
  self.assertTrue(first['valid'])
  # The old faster time estimate reused this path; the longer actual
  # approach puts its rear swing inside the future hard clearance region.
  self.assertNotEqual(first['path'],second['path'])
 def test_confirmed_moving_body_does_not_grow_with_filtered_velocity_lag(self):
  spans=[]
  for k in range(40):
   z=self.p.ask(stamp=1+k*.2,clusters=[cluster(1,1.8+k*.08)])
   o=z['occupancies'][0]
   if k>=10:
    self.assertTrue(o['has_moved']);spans.append(o['xmax']-o['xmin'])
  self.assertLess(max(spans)-min(spans),.005)
 def test_actual_return_heading_limits_coast_speed(self):
  q=[3.9595783347552493,-.016624216802456914,.3727378617334732]
  z=self.p.ask(state_coast_cap=True,poses=[q],servo=-.093,upper=2.)
  self.assertTrue(z['valid'],z);self.assertGreaterEqual(z['cap'],.5);self.assertLess(z['cap'],1.43)
  checked=self.p.ask(coast_map=True,poses=[q],servo=-.093,speed=z['cap'])
  self.assertTrue(checked['valid'],checked);self.assertGreaterEqual(checked['minimum_map_reserve_m'],.04)
 def test_exhausted_search_is_explicit_and_does_not_admit_a_path(self):
  q=json.loads((ROOT/'compact_profile_fixture_20261112.json').read_text())['input']
  z=self.p.ask(**q,search_budget_ms=0,cap=.95,max_offset=.55)
  self.assertFalse(z['valid']);self.assertEqual(z['path'],[])
  self.assertTrue(z['score_terms']['search_budget_exhausted'])
if __name__=='__main__':unittest.main(verbosity=2)
