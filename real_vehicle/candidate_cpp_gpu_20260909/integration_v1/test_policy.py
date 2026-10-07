import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import integration_policy as policy

def args(**kw):
    data=dict(offline_self_test=False,stand_confirmed=True,zero_only=True,allow_wheel_motion=False,
              duration=240.,require_localization=True,route_start_confirmed=False,zero_route_seed=True,pose_timeout=.20)
    data.update(kw);return SimpleNamespace(**data)

class PolicyTests(unittest.TestCase):
    def test_physical_motion_refused(self):
        with self.assertRaises(ValueError):policy.validate(args(zero_only=False,allow_wheel_motion=True,zero_route_seed=False),'85',':1',True)
    def test_domain_zero_refused(self):
        with self.assertRaises(ValueError):policy.validate(args(),'0',None,False)
    def test_requires_stand(self):
        with self.assertRaises(ValueError):policy.validate(args(stand_confirmed=False),'85',None,False)
    def test_requires_localization(self):
        with self.assertRaises(ValueError):policy.validate(args(require_localization=False),'85',None,False)
    def test_no_timeout_relaxation(self):
        with self.assertRaises(ValueError):policy.validate(args(pose_timeout=.30),'85',None,False)
    def test_offline_zero_allowed(self):
        policy.validate(args(offline_self_test=True,stand_confirmed=False,zero_route_seed=False),'83',None,False)
    def test_offline_fault_allowed(self):
        policy.validate(args(offline_self_test=True,stand_confirmed=False,zero_only=False,zero_route_seed=False,require_localization=False),'83',None,False)
    def test_requires_offline_evidence(self):
        with patch.object(policy,'verify_manifests',return_value={'fingerprint':'test'}),patch.object(policy,'offline_evidence',side_effect=ValueError('missing')) as evidence:
            with self.assertRaises(ValueError):policy.validate(args(),'85',None,False)
            evidence.assert_called_once_with('test')
    def test_zero_cpp_contract(self):
        source=(Path(__file__).parent/'src/candidate_zero_pp/node.cpp').read_text()
        self.assertIn('d.speed=0.;d.angle=0.;',source)
        self.assertIn('enabled=false;enable_time=mono()',source)
        self.assertNotIn('enabled=m->data',source)
    def test_fixture_domain_guard(self):
        source=(Path(__file__).parent/'stand_test_inputs.py').read_text()
        self.assertIn("os.environ.get('ROS_DOMAIN_ID')!='83'",source)
        ast.parse(source)

    def test_ground_pf_uses_unified_half_second_freshness(self):
        source=(Path(__file__).parent/'gpu_pf_zero.py').read_text()
        tree=ast.parse(source)
        self.assertIn('self.scan_clock.timeout = .50',source)
        self.assertIn('self.odom_clock.timeout = .50',source)
        self.assertIn("Candidate PF freshness budget must remain 0.50 seconds",source)
        self.assertIsInstance(tree,ast.Module)

if __name__=='__main__':unittest.main()
