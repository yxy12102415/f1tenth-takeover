#!/usr/bin/env python3
"""381 v2 commissioning entry: static avoidance and dynamic following."""
import argparse,json,subprocess,sys,hashlib
from pathlib import Path
import cpp_vehicle_session as session
RELEASE=session.WS/'candidate_cpp_gpu_20260909/ground_381_2_0_dynamic_avoidance_20261115_v2'
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('tracker',choices=['pid','dynamic_mpc'])
 p.add_argument('mode',choices=['status','check','zero','run','start'],nargs='?',default='status')
 args,extra=p.parse_known_args()
 if '--baseline' in extra:raise RuntimeError('This entry uses the v2 occupancy planner; baseline profiles have separate entries.')
 report=json.loads((RELEASE/'reports/dynamic_preparation/acceptance.json').read_text())
 config=json.loads((RELEASE/'compact_config.json').read_text())
 binaries_match=all((RELEASE/'install_cpp/cpp_frenet/lib/cpp_frenet'/name).is_file() and hashlib.sha256((RELEASE/'install_cpp/cpp_frenet/lib/cpp_frenet'/name).read_bytes()).hexdigest()==expected for name,expected in report['binaries'].items())
 sources_match=bool(report.get('sources')) and all((RELEASE/name).is_file() and hashlib.sha256((RELEASE/name).read_bytes()).hexdigest()==expected for name,expected in report['sources'].items())
 tracker_qualified=args.tracker in config.get('qualified_motion_trackers',[])
 if args.mode=='status':
  print(json.dumps(dict(release=str(RELEASE),tracker=args.tracker,software_381_passed=bool(report['passed'] and binaries_match and sources_match and tracker_qualified),binaries_match_acceptance=binaries_match,sources_match_acceptance=sources_match,tracker_qualified=tracker_qualified,full_dynamic_overtake_passed=report['full_dynamic_overtake_passed'],physical_commissioned=config.get('physical_commissioned',False),physical_commissioning_enabled=config.get('physical_commissioning_enabled',False)),indent=2));return
 if not sources_match:raise RuntimeError('Vehicle sources differ from software acceptance; finish qualification before launch.')
 for name,expected in report['binaries'].items():
  binary=RELEASE/'install_cpp/cpp_frenet/lib/cpp_frenet'/name
  if hashlib.sha256(binary.read_bytes()).hexdigest()!=expected:raise RuntimeError('Vehicle binary differs from software acceptance: '+name)
 if args.mode!='check' and (not report['passed'] or not config.get('physical_commissioning_enabled')):
  raise RuntimeError('This release is not enabled for physical commissioning; inspect its acceptance report.')
 if args.mode!='check' and args.tracker not in config.get('qualified_motion_trackers',[]):raise RuntimeError('Tracker is not qualified for this commissioning scope.')
 session.PID_RELEASE=RELEASE;session.JOINT_RELEASE=RELEASE
 print('381 v2 trial: stationary-opponent avoidance commissioning; full dynamic overtaking is experimental.',flush=True)
 print('Original 381 map, start and open route; command limits 0.6–2.0 m/s, positive acceleration <=1.0 m/s²; terminal command 0.',flush=True)
 sys.argv=[sys.argv[0],args.tracker,args.mode]+extra
 if not any(v=='--duration' or v.startswith('--duration=') for v in extra):sys.argv+=['--duration','240']
 session.main()
if __name__=='__main__':
 try:main()
 except (RuntimeError,OSError,subprocess.CalledProcessError) as e:
  print('REFUSED: '+str(e),file=sys.stderr);sys.exit(1)
