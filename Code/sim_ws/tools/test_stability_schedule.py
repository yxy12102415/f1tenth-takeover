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

    def test_unsafe_prefix_rejected(self):
        self.assertNotEqual(self.plan('--prefix', '../unsafe').returncode, 0)


if __name__ == '__main__':
    unittest.main()
