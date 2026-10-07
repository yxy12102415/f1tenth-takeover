"""Ground PP admission with fresh, matching offline and real zero evidence."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from controller_config import TRACKER
ROOT=Path(__file__).resolve().parent
CAND=ROOT.parent
BASE=CAND.parent/'baseline_pf_pp_20260908'
FAULTS={'window','release','guard_kill','window_kill','supervisor_permit_loss',
        'supervisor_main_stall','health_kill','health_stale','health_overcurrent','health_overtemp'}

def verify_manifests():
    paths=[CAND/'runtime_manifest.sha256',BASE/'groundprep_runtime_v3_manifest.sha256',
           CAND/'integration_v1/runtime_manifest.sha256',ROOT/'runtime_manifest.sha256']
    for p in paths:
        result=subprocess.run(['sha256sum','-c',str(p)],cwd=str(p.parent),capture_output=True,text=True,timeout=10)
        if result.returncode:raise ValueError('Changed/unbuilt runtime: '+result.stdout+result.stderr)
    return dict(fingerprint=hashlib.sha256(b'\n'.join(p.read_bytes() for p in paths)+b'\ntracker='+TRACKER.encode()).hexdigest(),
                manifests=[str(p) for p in paths],physical_motion_scope='one_ground_fitzpatrick_frenet_cpp_'+TRACKER+'_single_lap_max_90s_0.70_to_2.00mps',
                max_steering_rad=.27,route_completion_requires_logged_endpoint=True)

def evidence_status(reports,identity,now):
    offline_zero=False;physical_zero=False;faults=set()
    for r in reports:
        if not isinstance(r,dict) or r.get('passed') is not True or r.get('code_fingerprint')!=identity:continue
        t=r.get('completed_unix')
        if not isinstance(t,(int,float)) or not math.isfinite(t) or not 0<=now-t<=86400:continue
        if r.get('pose_timeout_sec')!=.50 or r.get('power_mode_record')!='pmode:0008':continue
        seconds=r.get('localization_seconds')
        zero=(r.get('zero_only') is True and r.get('motion_permission') is False
              and r.get('continuous_localization_passed') is True
              and isinstance(seconds,(int,float)) and math.isfinite(seconds) and seconds>=5.)
        if r.get('offline') is True:
            offline_zero |= zero
            if r.get('zero_only') is False and r.get('fault') in FAULTS:faults.add(r['fault'])
        elif r.get('offline') is False and now-t<=900:
            physical_zero |= (zero and r.get('physical_mode')=='ground'
                and r.get('zero_joy') is True and r.get('zero_rviz') is True
                and r.get('initialization_mode')=='operator_confirmed_route_start'
                and r.get('supervisor_lease_expired') is False
                and r.get('timing',{}).get('max_abs_erpm')==0.)
    return offline_zero,faults,physical_zero

def targeted_evidence(reports,identity,now):
    return any(r.get('passed') is True and r.get('offline') is True
        and r.get('code_fingerprint')==identity and r.get('fault')=='window'
        and r.get('max_command_speed_mps')==2.00
        and r.get('health_source_timeout_sec')==.50
        and r.get('health_receipt_timeout_sec')==.50
        and isinstance(r.get('completed_unix'),(int,float))
        and 0<=now-r['completed_unix']<=86400
        and r.get('first_actual_rpm_command',1e9)<=150.001
        and r.get('fixture_stop_after_observation_ms',1e9)<=500
        and r.get('supervisor_lease_expired') is False for r in reports)

def require_evidence(identity,motion):
    reports=[]
    for p in (ROOT/'run_logs').glob('*/result.json'):
        try:reports.append(json.loads(p.read_text()))
        except (ValueError,OSError):continue
    now=time.time();zero,faults,physical=evidence_status(reports,identity,now)
    targeted=targeted_evidence(reports,identity,now)
    if not targeted and (not zero or faults!=FAULTS):
        raise ValueError('Need matching targeted 500ms integration evidence or full offline suite')
    if motion and not physical:raise ValueError('Need same-release ground 5s zero with Joy/RViz and confirmed route start within 15 minutes')

def validate(args,domain,display,tty):
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('Localhost only required')
    if not math.isfinite(args.duration) or not 10<=args.duration<=300:raise ValueError('Invalid session duration')
    if args.pose_timeout!=.50:raise ValueError('Pose timeout remains .50s')
    if args.offline_self_test:
        if domain!='83' or any((args.ground_confirmed,args.cutoff_confirmed,args.route_clear_confirmed,
                               args.allow_wheel_motion,args.route_start_confirmed,args.zero_route_seed)):
            raise ValueError('Offline mode must contain no physical confirmations')
        if args.zero_only and not args.require_localization:raise ValueError('Offline zero requires localization')
        return
    acceptance = ROOT/'run_logs/fitzpatrick_single_lap_validation.json'
    if not acceptance.exists():raise ValueError('Fitzpatrick single-lap validation missing')
    checked=json.loads(acceptance.read_text())
    if checked.get('passed') is not True or checked.get('manifest_sha256')!=hashlib.sha256((ROOT/'runtime_manifest.sha256').read_bytes()).hexdigest():
        raise ValueError('Need matching Fitzpatrick single-lap model/ROS validation')
    if domain!='85' or not all((args.ground_confirmed,args.cutoff_confirmed,args.route_clear_confirmed,args.route_start_confirmed)):
        raise ValueError('Confirm ground, cutoff, clear route plus stopping buffer, and actual route-start pose')
    if args.zero_route_seed:raise ValueError('Unverified route seed forbidden')
    if args.allow_wheel_motion==args.zero_only:raise ValueError('Choose zero-only OR allow-ground-motion')
    if args.zero_only and not all((args.require_localization,args.zero_rviz,args.zero_joy)):
        raise ValueError('Ground zero must include localization, real Joy and RViz')
    if args.allow_wheel_motion and (not display or not tty):raise ValueError('Desktop and interactive terminal required')
    if args.fault!='window':raise ValueError('Real fault injection forbidden')
    require_evidence(verify_manifests()['fingerprint'],args.allow_wheel_motion)


def zero_health_ready(statuses,received,now):
    return (all(0.<=now-received.get(k,-1e9)<.50 for k in ('health_guard','window'))
            and statuses.get('health_guard',{}).get('healthy') is True
            and statuses.get('window',{}).get('health_ready') is True
            and statuses.get('window',{}).get('latched') is False)
