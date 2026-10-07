"""Regression wrapper for the original turn-entry failure under the dynamic observer."""
import json
from test_observer_replay import ROOT, check
if __name__=='__main__':
 source=ROOT.with_name('ground_fitzpatrick_joint_20260927_v4')/'run_logs/20260927_215927-closed-frenet-ground-attempt/tracking_trace.json'
 report=check(source)
 (ROOT/'run_logs/rate_replay.json').write_text(json.dumps(report,indent=2)+'\n')
 print('Original turn-entry replay passed:',report['frames'])
