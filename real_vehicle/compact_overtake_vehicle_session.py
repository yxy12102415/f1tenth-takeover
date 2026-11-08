#!/usr/bin/env python3
"""Select the isolated, bounded 381 2.0 straight overtaking experiment."""
import json
import subprocess
import sys
import cpp_vehicle_session as session

session.JOINT_RELEASE = session.WS / 'candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261108_v5'
try:
    config=json.loads((session.JOINT_RELEASE/'compact_config.json').read_text())
    if not config.get('physical_commissioning_enabled',False):
        raise RuntimeError('Compact overtaking physical commissioning is not enabled; software/sensor acceptance is pending.')
    print('Compact v5 (input gate fix): fixed right-side pass 0.55m + C++ dynamic MPC; 5.95m endpoint; lead center 1.80m. '
          'F1 handset=B4; lead handset=A3, 1150us. '
          'Keep both L1 released until READY; F1 L1 continuous >=1s; after F1 starts, lead L1 about 3s then release.',flush=True)
    sys.argv=[sys.argv[0],'dynamic_mpc']+(sys.argv[1:] or ['run'])
    if not any(arg=='--duration' or arg.startswith('--duration=') for arg in sys.argv):sys.argv+=['--duration','240']
    session.main()
except (RuntimeError,OSError,subprocess.CalledProcessError) as error:
    print('REFUSED: '+str(error),file=sys.stderr);sys.exit(1)
