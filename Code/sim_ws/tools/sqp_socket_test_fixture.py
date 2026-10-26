#!/usr/bin/env python3
"""Fault-injection fixture; never launched by a simulation or hardware node."""
import json
import sys
import time

print('{"ready":true}', flush=True)
for i, line in enumerate(sys.stdin):
    request = json.loads(line)
    if i == 0:
        # Deliberately split a response across the caller's deadline.
        sys.stdout.write('{"ok":true,'); sys.stdout.flush()
        time.sleep(.060)
        print('"token":"STALE"}', flush=True)
    else:
        print(json.dumps(dict(ok=True, token=request['token'])), flush=True)
