"""Read-only ground preflight. No ROS initialization, devices opened, or motion."""
import argparse
import json
from pathlib import Path
import time
from ground_readiness import ROOT, evaluate, fingerprint


def select_evidence(log_root, expected):
    """Choose recent successful matching-code evidence; never convert a failure."""
    selected={};paths={}
    for path in sorted(log_root.glob('*-pf-pp-stand-*/result.json')):
        try:r=json.loads(path.read_text())
        except (OSError,ValueError):continue
        if not isinstance(r,dict) or r.get('code_fingerprint')!=expected or r.get('offline') is not False:
            continue
        if r.get('passed') is not True:continue
        if r.get('zero_only') is True:
            name='zero'
        else:
            w=r.get('window');f=r.get('final')
            if not isinstance(w,dict) or not isinstance(f,dict):continue
            reason=w.get('reason')
            if reason=='two_second_limit':name='window'
            elif reason=='deadman_release' or (reason=='upstream_stop' and f.get('stop_reason')=='teleop_deadman_released'):name='release'
            else:continue
        # The full checker subsequently enforces age, duration, motor evidence
        # and initialization provenance. A short timing pass is not acceptance.
        selected[name]=r;paths[name]=str(path.parent)
    return selected,paths


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--json',action='store_true')
    args=p.parse_args();expected=fingerprint()
    reports,paths=select_evidence(ROOT/'run_logs',expected)
    result=evaluate(reports,expected,time.time())
    result.update(code_fingerprint=expected,evidence=paths,
        hardware_started=False,ground_executor_released=False,
        route_start_confirmation='Position and heading can be confirmed once with --route-start-confirmed; live localization is still checked.')
    if args.json:print(json.dumps(result,indent=2,ensure_ascii=False))
    else:
        print('GROUND MOTION NOT RELEASED. Read-only preflight; no hardware started.')
        for issue in result['blockers']:print('BLOCKED: '+issue)
        for requirement in result['manual_requirements']:print('REQUIRED: '+requirement)
        print('Stand commands are not ground commands. Do not pass --stand-confirmed with wheels on the ground.')
    # Even complete automated reports cannot attest physical coast distance,
    # firmware timeout or a reviewed ground executor. Never silently delegate
    # to the stand runner or add a force/bypass flag.
    raise SystemExit(2)


if __name__=='__main__':main()
