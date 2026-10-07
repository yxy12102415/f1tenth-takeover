"""Read-only evidence checker. NEVER starts ROS/drivers or authorizes motion."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parent
FILES=('stand_follow.py','stand_window.py','stand_readiness.py','stand_scan_relay.py',
       'stand_map_io.py','stand_policy.py','start_pf_pp_stand.sh','clean_env.sh',
       'ground_readiness.py','check_runtime.py','stand_telemetry.py',
       'config/pf_stationary_candidate.yaml')


def fingerprint(root=ROOT):
    files=[root/name for name in FILES]
    files+=[root/'install_groundprep_v3/baseline_pf_pp/lib/python3.8/site-packages/baseline_pf_pp'/name
            for name in ('core.py','follower.py','safe_pf.py','runtime_budget.py')]
    files+=sorted((root.parent/'route_install_v12/f1tenth_safety_migration/lib/python3.8/site-packages/f1tenth_safety_migration').glob('*.py'))
    files+=[root.parent/'maps/ps4_slam_20260904_163930.yaml',
            root.parent/'maps/ps4_slam_20260904_163930.pgm',
            root.parent/'routes/manual_route_20260904_215501_controller_ready.csv']
    digest=hashlib.sha256()
    for path in files:
        digest.update(str(path.relative_to(root.parent)).encode()+b'\0')
        digest.update(path.read_bytes())
    return digest.hexdigest()


def evaluate(reports,expected,now):
    issues=[]
    for name in ('zero','window','release'):
        r=reports.get(name)
        if not isinstance(r,dict):issues.append(name+': missing report');continue
        r=dict(r)
        for field in ('health','timing','window','final','localization_monitor'):
            if not isinstance(r.get(field,{}),dict):
                issues.append(name+': malformed '+field);r[field]={}
        if r.get('code_fingerprint')!=expected:issues.append(name+': not current code/map/route')
        if r.get('initialization_mode')=='unverified_route_seed':
            issues.append(name+': route-seeded timing diagnostic is not verified physical alignment')
        if r.get('pose_timeout_sec')!=.3:issues.append(name+': not the reviewed 0.30s experiment')
        if r.get('supervisor_lease_expired') is not False:issues.append(name+': supervisor lease not verified healthy')
        completed=r.get('completed_unix')
        if not isinstance(completed,(int,float)) or not math.isfinite(completed) or not 0.<=now-completed<=86400.:
            issues.append(name+': report missing timestamp or older than 24h')
        if r.get('passed') is not True or r.get('offline') is not False:
            issues.append(name+': no successful physical test')
        h=r.get('health',{})
        erpm=h.get('erpm');peak=r.get('timing',{}).get('max_abs_erpm')
        if not isinstance(erpm,(int,float)) or not math.isfinite(erpm) or abs(erpm)>=100. or h.get('fault')!=0:
            issues.append(name+': no stationary fault-free final telemetry')
        if name=='zero':
            if not isinstance(peak,(int,float)) or not math.isfinite(peak) or peak>=100.:
                issues.append('zero: stationary telemetry not verified throughout')
            seconds=r.get('localization_seconds',0.);samples=r.get('localization_monitor',{}).get('pp_samples',0)
            if (r.get('zero_only') is not True or r.get('motion_permission') is not False
                    or r.get('continuous_localization_passed') is not True
                    or not isinstance(seconds,(int,float)) or not math.isfinite(seconds) or seconds<120.
                    or not isinstance(samples,int) or samples<1000
                    or r.get('zero_joy') is not True or r.get('zero_rviz') is not True):
                issues.append('zero: requires >=120s continuous PF/PP with Joy and RViz, no motion permission')
        else:
            if (r.get('zero_only') is not False or r.get('motion_permission') is not True
                    or not isinstance(peak,(int,float)) or not math.isfinite(peak) or peak<100.):
                issues.append(name+': no verified physical wheel-motion telemetry')
            reason=r.get('window',{}).get('reason')
            final_reason=r.get('final',{}).get('stop_reason')
            if name=='window' and reason!='two_second_limit':issues.append('window: stopped early, not a completed bounded attempt')
            if name=='release' and not (reason=='deadman_release' or
                    (reason=='upstream_stop' and final_reason=='teleop_deadman_released')):
                issues.append('release: no unambiguous L1 release STOP')
    return {'automated_prerequisites_passed':not issues,'ground_motion_authorized':False,
            'blockers':issues,'manual_requirements':[
                'Reconfirm independent physical cutoff, battery and closed test area on the day.',
                'Verify localization during real translation with propulsion isolated; stationary/stand does not establish this.',
                'Measure physical coast stopping distance; virtual zero-RPM is not a physical braking measurement.',
                'Verify the real actuator/firmware watchdog with the actuator guard or host unavailable; the virtual sink watchdog is not evidence.',
                'Review route and a separate bounded ground launcher before any ground motion; stand launcher is forbidden on ground.']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('zero','window','release'):parser.add_argument('--'+name+'-log',type=Path)
    args=parser.parse_args();reports={}
    for name in ('zero','window','release'):
        path=getattr(args,name+'_log')
        if path is not None:
            try:reports[name]=json.loads((path/'result.json').read_text())
            except (OSError,ValueError):reports[name]=None
    result=evaluate(reports,fingerprint(),time.time())
    print(json.dumps(result,indent=2,ensure_ascii=False))
    raise SystemExit(0 if result['automated_prerequisites_passed'] else 2)


if __name__=='__main__':main()
