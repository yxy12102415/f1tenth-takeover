"""Summarize retained integration evidence; never opens hardware or starts nodes."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent

def summarize():
    rows=[]
    for path in sorted((ROOT.parent/'run_logs').glob('*/result.json')):
        r=json.loads(path.read_text())
        trace_path=path.with_name('command_trace.json')
        trace=json.loads(trace_path.read_text()) if trace_path.exists() else {}
        current=trace.get('current',[]);rpm=trace.get('rpm_command',[])
        final=trace.get('final_command',[]);virtual=trace.get('virtual_rpm',[])
        row={'log':str(path.parent),'fingerprint':r.get('code_fingerprint'),
             'passed':r.get('passed'),'offline':r.get('offline'),'zero_only':r.get('zero_only'),
             'fault':r.get('fault'),'motion_permission':r.get('motion_permission'),
             'localization_seconds':r.get('localization_seconds'),
             'current_frames_including_shutdown':len(current),'nonzero_current_frames':sum(v!=0 for _,v in current),
             'rpm_command_frames':len(rpm),'nonzero_rpm_frames':sum(v!=0 for _,v in rpm),
             'final_frames':len(final),'nonzero_final_frames':sum(v!=0 or a!=0 for _,v,a in final),
             'virtual_nonzero_frames':sum(v!=0 for _,v in virtual),
             'coast_command_after_fault_ms':r.get('coast_command_after_fault_ms'),
             'virtual_stop_after_fault_ms':r.get('stop_after_fault_ms'),
             'virtual_nonzero_span_ms':r.get('virtual_nonzero_span_ms'),
             'max_abs_erpm':r.get('timing',{}).get('max_abs_erpm'),
             'max_health_source_age_s':r.get('timing',{}).get('max_source_age_s'),
             'max_pose_source_age_s':r.get('localization_monitor',{}).get('max_pose_source_age_s')}
        if r.get('health'):row['health_before_shutdown']=r['health']
        if r.get('error'):
            row['error_prefix']=r['error'][:100]
            row['window_after_shutdown']=r.get('last_statuses',{}).get('window')
            if r.get('localization_failure'):row['localization_failure']=r['localization_failure']
        rows.append(row)
    (ROOT/'results_summary.json').write_text(json.dumps(rows,indent=2)+'\n')
    print(json.dumps(rows,indent=2))
if __name__=='__main__':summarize()
