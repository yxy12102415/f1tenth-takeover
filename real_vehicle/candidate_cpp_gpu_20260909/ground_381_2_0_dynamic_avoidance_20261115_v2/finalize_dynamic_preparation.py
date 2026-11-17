"""Summarize actual software evidence; never enable physical motion."""
import json,hashlib,math
from pathlib import Path
R=Path(__file__).resolve().parent
OUT=R/'reports/dynamic_preparation'

def read(p):return json.loads(p.read_text())
def main():
 tests=[]
 for tracker in ('pid','dynamic_mpc'):
  for case in ('clear','static','moving','scan_timeout'):
   d=OUT/(tracker+'_'+case);p=d/'report.json'
   item=dict(name=tracker+'_'+case,report=str(p.relative_to(R)),passed=False)
   if p.exists():
    v=read(p);item.update(passed=v['passed'],failure=v['failure'],route_completed=v['route_completed'],minimum_body_separation_m=v['minimum_independent_sat_separation_m'],max_command_mps=v['max_command_mps'])
    if (d/'records.json').exists():
     c=read(d/'records.json')['controller'];item['following_used']=any(v.get('state')=='FOLLOWING_OBSTACLE' for v in c)
     running=[v.get('speed_command') for v in c if v.get('valid') and (v.get('tracker_diagnostics') or {}).get('speed_phase')=='driving']
     item['minimum_running_command_mps']=min(running) if running else None
     item['minimum_command_check_passed']=bool(running) and min(running)>=.5-1e-5
     acceleration=[(v.get('tracker_diagnostics') or {}).get('acceleration_command_mps2',0.) for v in c if v.get('valid')]
     item['maximum_positive_acceleration_command_mps2']=max(acceleration,default=0.)
     item['acceleration_command_check_passed']=max(acceleration,default=0.)<=1.00001
     item['speed_command_check_passed']=item['max_command_mps']<=2.00001
     if case!='scan_timeout':item['passed']=item['passed'] and item['minimum_command_check_passed'] and item['acceleration_command_check_passed'] and item['speed_command_check_passed']
   tests.append(item)
  d=OUT/'long_straight_fixture/reports'/(tracker+'_moving_long');p=d/'report.json';item=dict(name=tracker+'_long_dynamic_overtake',report=str(p.relative_to(R)),passed=False)
  if p.exists():
   v=read(p);records=read(d/'records.json');last=records['plant'][-1];passed_lead=last['x']>last['lead_x']+.44
   item.update(passed=v['passed'] and passed_lead,route_completed=v['route_completed'],fully_passed_lead=passed_lead,failure=v['failure'],minimum_body_separation_m=v['minimum_independent_sat_separation_m'],max_command_mps=v['max_command_mps'])
  tests.append(item)
 native=read(OUT/'native_memory.json')
 build_time=(R/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_controller').stat().st_mtime
 for v in tests:
  v['required_for_381_avoidance']='long_dynamic_overtake' not in v['name']
  evidence=R/v['report'];v['report_is_current']=evidence.exists() and evidence.stat().st_mtime>=build_time
  v['passed']=v['passed'] and v['report_is_current']
 required=[v for v in tests if v['required_for_381_avoidance']]
 experimental=[v for v in tests if not v['required_for_381_avoidance']]
 captured=read(OUT/'captured_memory_replay.json')
 passed=native['passed'] and captured['passed'] and all(v['passed'] for v in required)
 full_overtake_passed=all(v['passed'] for v in experimental)
 a=dict(scope='Software preparation only: synthetic ROS sensors/plant plus native regressions; no physical motion',passed=passed,native_passed=native['passed'],tests=tests,physical_motion=False,physical_commissioned=False)
 a['full_dynamic_overtake_passed']=full_overtake_passed
 a['complete_overtaking_is_experimental']=not full_overtake_passed
 a['captured_real_lidar_memory_replay_passed']=captured['passed']
 a['sources']={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((R/'src/cpp_frenet').glob('*')) if p.is_file()}
 a['binaries']={name:hashlib.sha256((R/'install_cpp/cpp_frenet/lib/cpp_frenet'/name).read_bytes()).hexdigest() for name in ['cpp_frenet_planner','cpp_frenet_controller']}
 a['limits']=dict(max_speed_mps=2.,minimum_running_command_mps=.5,positive_acceleration_limit_mps2=1.,sensor_timeout_s=.5,obstacle_velocity_tested_mps=[.2,.5,.65])
 (OUT/'acceptance.json').write_text(json.dumps(a,ensure_ascii=False,indent=2)+'\n')
 s=read(R/'DYNAMIC_PREPARATION_STATUS.json');s['software_acceptance_passed']=passed;s['evidence']='reports/dynamic_preparation/acceptance.json';s['software_qualified_trackers']=['pid','dynamic_mpc'] if passed else [];s['physical_motion_started']=bool(s.get('physical_motion_started',False))
 for key in s['items']:s['items'][key]='implemented_software_verified' if passed else 'implemented_acceptance_pending'
 s['full_dynamic_overtake_passed']=full_overtake_passed
 s['remaining']=['Revised compact-departure candidate has not been physically tested; previous field run stopped mid-route without contact.']+[v['name'] for v in tests if not v['passed']]
 s['current_revision_status']='software_verified_physical_retest_pending' if passed else 'software_acceptance_incomplete'
 (R/'DYNAMIC_PREPARATION_STATUS.json').write_text(json.dumps(s,ensure_ascii=False,indent=2)+'\n')
 lines=['# Dynamic avoidance software acceptance','', '381 avoidance preparation passed: '+str(passed), 'Full dynamic overtaking passed: '+str(full_overtake_passed),'','Software checks use no physical motion. The previous field run stopped mid-route without contact; the revised candidate has not been physically tested.','', '| Case | Passed | Completed route | Following / full pass |','|---|---|---|---|']
 for v in tests:lines.append('| '+v['name']+' | '+str(v['passed'])+' | '+str(v.get('route_completed'))+' | '+str(v.get('fully_passed_lead',v.get('following_used')))+' |')
 lines+=['','The 381 route is unchanged. The 15 m straight is a separate synthetic fixture. A completed route while following is not labelled as a completed overtake.','', 'References and change rationale: [OPEN_SOURCE_REVIEW.md](../../OPEN_SOURCE_REVIEW.md).']
 (OUT/'ACCEPTANCE.md').write_text('\n'.join(lines)+'\n')
 print(json.dumps(dict(passed=passed,tests=[dict(name=v['name'],passed=v['passed'],failure=v.get('failure')) for v in tests]),indent=2))
if __name__=='__main__':main()
