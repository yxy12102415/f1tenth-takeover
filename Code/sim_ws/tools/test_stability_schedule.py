#!/usr/bin/env python3
"""Read-only schedule tests; never launches ROS or generates trial output."""
import collections
import json
from pathlib import Path
import subprocess
import sys
import unittest

SCRIPT = Path(__file__).with_name('run_stability_comparison.py')


class Schedule(unittest.TestCase):
    def plan(self, *extra):
        return subprocess.run([sys.executable, str(SCRIPT), '--prefix', 'test_schedule',
            '--plan-only', *extra], capture_output=True, text=True)

    def test_reproducible_and_paired(self):
        a, b = self.plan('--repeats', '2'), self.plan('--repeats', '2')
        self.assertEqual(a.returncode, 0)
        self.assertEqual(a.stdout, b.stdout)
        schedule = json.loads(a.stdout)['schedule']
        self.assertEqual(len(schedule), 16)
        blocks = collections.defaultdict(list)
        destinations = set()
        for row in schedule:
            blocks[row['scenario'], row['seed']].append(row['progressive_pass_speed'])
            destinations.add((row['scenario'], row['run_id']))
        self.assertEqual(len(destinations), 16)
        self.assertEqual(len(blocks), 8)
        for values in blocks.values():
            self.assertEqual(sorted(values), [False, True])

    def test_invalid_repeats(self):
        self.assertNotEqual(self.plan('--repeats', '0').returncode, 0)

    def test_fast_geometry_pairs_current_profile(self):
        result=self.plan('--fast-geometry-comparison')
        self.assertEqual(result.returncode,0,result.stderr)
        schedule=json.loads(result.stdout)['schedule']
        self.assertEqual(len(schedule),8)
        for i in range(0,8,2):
            pair=schedule[i:i+2]
            self.assertEqual(pair[0]['scenario'],pair[1]['scenario'])
            self.assertEqual(sorted(r['fast_pass_geometry'] for r in pair),[False,True])
            self.assertTrue(all(r['predictive_speed_profile'] and not r['progressive_pass_speed'] for r in pair))

    def test_fast_response_pairs_geometry_only(self):
        result=self.plan('--fast-response-comparison')
        self.assertEqual(result.returncode,0,result.stderr)
        schedule=json.loads(result.stdout)['schedule']
        for i in range(0,8,2):
            pair=schedule[i:i+2]
            self.assertEqual(pair[0]['scenario'],pair[1]['scenario'])
            self.assertEqual(sorted(r['fast_pass_response'] for r in pair),[False,True])
            self.assertTrue(all(r['predictive_speed_profile'] and r['fast_pass_geometry'] and
                                not r['progressive_pass_speed'] for r in pair))
        self.assertNotEqual(self.plan('--fast-response-comparison','--profile-comparison').returncode,0)

    def test_verified_exit_pairs_geometry_only_without_fast_response(self):
        result=self.plan('--verified-exit-comparison')
        self.assertEqual(result.returncode,0,result.stderr)
        schedule=json.loads(result.stdout)['schedule']
        self.assertEqual(len(schedule),8)
        for i in range(0,8,2):
            pair=schedule[i:i+2]
            self.assertEqual(pair[0]['scenario'],pair[1]['scenario'])
            self.assertEqual(sorted(r['verified_exit_recovery'] for r in pair),[False,True])
            self.assertTrue(all(r['predictive_speed_profile'] and r['fast_pass_geometry'] and
                               not r['fast_pass_response'] and not r['progressive_pass_speed'] for r in pair))

    def test_unsafe_prefix_rejected(self):
        self.assertNotEqual(self.plan('--prefix', '../unsafe').returncode, 0)

    def test_control_recovery_is_the_only_different_policy(self):
        result=self.plan('--control-recovery-comparison')
        self.assertEqual(result.returncode,0,result.stderr)
        schedule=json.loads(result.stdout)['schedule']
        for i in range(0,8,2):
            pair=schedule[i:i+2]
            self.assertEqual(pair[0]['scenario'],pair[1]['scenario'])
            self.assertEqual(sorted(r['checked_control_recovery'] for r in pair),[False,True])
            self.assertTrue(all(r['predictive_speed_profile'] and r['fast_pass_geometry'] and
                               r['verified_exit_recovery'] and not r['fast_pass_response'] and
                               not r['progressive_pass_speed'] for r in pair))
        self.assertNotEqual(self.plan('--control-recovery-comparison','--verified-exit-comparison').returncode,0)


if __name__ == '__main__':
    unittest.main()
