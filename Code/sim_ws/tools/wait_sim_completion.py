#!/usr/bin/env python3
"""Bounded wall-clock supervision of a physics-time trial; never controls ROS."""
import json
import os
from pathlib import Path
import sys
import time


def wait(summary, duration, pid):
    deadline = time.monotonic() + 4 * duration + 60
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            raise SystemExit('ROS launch ended before the simulation-time target')
        if summary.exists():
            value = json.loads(summary.read_text())
            if value.get('duration_s', 0) >= duration:
                return
        time.sleep(.2)
    raise SystemExit('Simulation-time target not reached within bounded wall-clock budget')


if __name__ == '__main__':
    wait(Path(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3]))
