import unittest
from stand_readiness import ContinuousZeroQualification
class ZeroQualification(unittest.TestCase):
 def test_fault_between_observations_breaks_credited_interval(self):
  q=ContinuousZeroQualification(5)
  self.assertFalse(q.update(True,0,0));self.assertFalse(q.update(True,4.9,0))
  self.assertFalse(q.update(True,5.1,1));self.assertFalse(q.update(True,10.,1))
  self.assertTrue(q.update(True,10.1,1))
 def test_invalid_state_does_not_accumulate_stability(self):
  q=ContinuousZeroQualification(5)
  self.assertFalse(q.update(True,0,0));self.assertFalse(q.update(False,4,0))
  self.assertFalse(q.update(True,7,0));self.assertFalse(q.update(True,11.9,0));self.assertTrue(q.update(True,12,0))
 def test_backward_or_nonfinite_clock_cannot_qualify(self):
  for now in (float('nan'),-1):
   q=ContinuousZeroQualification(5);q.update(True,0,0)
   with self.assertRaises(ValueError):q.update(True,now,0)
if __name__=='__main__':unittest.main(verbosity=2)
