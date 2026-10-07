import unittest
from unittest.mock import patch
from types import SimpleNamespace
from gpu_pf_offline import Candidate, SafeParticleFilter

class PolicyTests(unittest.TestCase):
    def test_empty_batch_rejected(self):self.rejected(0)
    def test_oversize_batch_rejected(self):self.rejected(3156)
    def rejected(self,count):
        calls=[]
        fake=SimpleNamespace(ANGLE_STEP=12,MAX_PARTICLES=1000,
                             fault=lambda why:calls.append(why),health=lambda:calls.append('health'))
        with patch.object(SafeParticleFilter,'lidarCB') as downstream:
            Candidate.lidarCB(fake,SimpleNamespace(ranges=[1.]*count))
            downstream.assert_not_called()
        self.assertEqual(calls,['gpu_batch_outside_reviewed_capacity','health'])
    def test_1081_ray_scan_accepted(self):
        fake=object.__new__(Candidate);fake.ANGLE_STEP=12;fake.MAX_PARTICLES=1000
        msg=SimpleNamespace(ranges=[1.]*1081)
        with patch.object(SafeParticleFilter,'lidarCB') as downstream:
            Candidate.lidarCB(fake,msg);downstream.assert_called_once_with(msg)
    def test_unreviewed_profiles_rejected(self):
        base={'MAX_PARTICLES':1000,'WHICH_RM':'rmgpu','RANGELIB_VAR':2,'ANGLE_STEP':12}
        for key,value in [('MAX_PARTICLES',4000),('WHICH_RM','cddt'),('RANGELIB_VAR',3),('ANGLE_STEP',1)]:
            values=dict(base);values[key]=value
            def init(obj):
                for k,v in values.items():setattr(obj,k,v)
            with patch.object(SafeParticleFilter,'__init__',init):
                with self.assertRaises(RuntimeError):Candidate()

if __name__=='__main__':unittest.main()
