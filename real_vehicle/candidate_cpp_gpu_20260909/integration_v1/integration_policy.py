"""Independent integration policy. No physical motion branch is releasable."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
CAND=ROOT.parent
BASE=CAND.parent/'baseline_pf_pp_20260908'
sys.path.insert(0,str(BASE))
from stand_policy import validate as baseline_validate

def verify_manifests():
    manifests=[CAND/'runtime_manifest.sha256',BASE/'groundprep_runtime_v3_manifest.sha256',ROOT/'runtime_manifest.sha256']
    for manifest in manifests:
        result=subprocess.run(['sha256sum','-c',str(manifest)],cwd=str(manifest.parent),capture_output=True,text=True,timeout=10)
        if result.returncode:raise ValueError('Runtime changed or unbuilt: '+result.stdout+result.stderr)
    # Manifest contents are immutable identities; sha256sum above verifies bytes.
    identity=hashlib.sha256(b'\n'.join(p.read_bytes() for p in manifests)).hexdigest()
    return {'fingerprint':identity,'manifests':[str(p) for p in manifests],'physical_motion_released':False}

def offline_evidence(identity):
    zero=False;faults=set()
    for path in (CAND/'run_logs').glob('*/result.json'):
        try:r=json.loads(path.read_text())
        except (ValueError,OSError):continue
        if r.get('code_fingerprint')!=identity or not r.get('offline') or not r.get('passed'):continue
        if r.get('zero_only') and r.get('continuous_localization_passed') and r.get('localization_seconds',0)>=120.:zero=True
        if not r.get('zero_only') and r.get('fault') in ('window','release','guard_kill','window_kill','supervisor_permit_loss','supervisor_main_stall'):
            faults.add(r['fault'])
    missing=set(('window','release','guard_kill','window_kill','supervisor_permit_loss','supervisor_main_stall'))-faults
    if not zero or missing:raise ValueError('Physical ZERO blocked: need matching 120s offline zero and all six full-chain faults; zero='+str(zero)+' missing='+','.join(sorted(missing)))

def validate(args,domain,display,tty):
    if args.pose_timeout!=.20:raise ValueError('Candidate C++ pose timeout is fixed at 0.20s')
    if args.offline_self_test and args.zero_only:
        if not args.require_localization:raise ValueError('Offline zero requires localization monitoring')
        proxy=copy.copy(args);proxy.zero_only=False
        baseline_validate(proxy,domain,display,tty)
    else:baseline_validate(args,domain,display,tty)
    if not args.offline_self_test:
        if args.allow_wheel_motion or not args.zero_only:raise ValueError('This integration releases physical ZERO ONLY, no wheel motion')
        if not args.require_localization:raise ValueError('Physical zero must monitor localization')
        offline_evidence(verify_manifests()['fingerprint'])
