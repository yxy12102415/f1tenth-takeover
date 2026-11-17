from pathlib import Path
import json,math
import numpy as np
R=Path(__file__).resolve().parents[1];f=json.loads((R/'review/dmpc_20261116_155527_moving_failure.json').read_text());trace=json.loads((R/'run_logs/20261116_155527-closed-frenet-ground-attempt/tracking_trace.json').read_text());d=f['candidate_collision_diagnostics'][0];o=d['occupancy'];row=next(x for x in trace['controller'] if x.get('valid') and x.get('input_scan_stamp')==o['stamp'] and x.get('obstacle_points'));points=np.array(row['obstacle_points']);points=points[(points[:,0]>=o['xmin']-1e-6)&(points[:,0]<=o['xmax']+1e-6)&(points[:,1]>=o['ymin']-1e-6)&(points[:,1]<=o['ymax']+1e-6)];path=np.array(next(x['local_path_snapshot'] for x in trace['controller'] if x.get('plan_id')==row['plan_id'] and x.get('local_path_snapshot')));route=np.array(json.loads((R/'runtime_assets.json').read_text())['route']);q=np.array(d['ego_pose']);lo=np.array(d['predicted_box'][:2]);hi=np.array(d['predicted_box'][2:]);extra=d['clearance_m']
def distances(points,pose,hx=.29,hy=.155):
 c,s=np.cos(pose[2]),np.sin(pose[2]);local=(points-pose[:2])@np.array([[c,-s],[s,c]]);return np.linalg.norm(np.maximum(np.abs(local)-[hx,hy],0),axis=1)
def dense(path):
 out=[]
 for a,b in zip(path[:-1],path[1:]):
  n=max(1,int(math.ceil(np.linalg.norm(b[:2]-a[:2])/.005)));out.extend(a+(b-a)*x/n for x in range(n))
 return out+[path[-1]]
def ptseg(p,a,b):
 delta=b-a;t=np.clip(np.sum((p-a)*delta,axis=-1)/np.sum(delta*delta),0,1);return np.linalg.norm(p-(a+t[...,None]*delta),axis=-1)
def center_distance(path):return min(float(np.min(ptseg(points,a[:2],b[:2]))) for a,b in zip(path[:-1],path[1:]))
def bodydistance(path):return min(float(distances(points,p).min()) for p in dense(path))
def corners(q,hx,hy):
 c,s=math.cos(q[2]),math.sin(q[2]);return np.array([[-hx,-hy],[hx,-hy],[hx,hy],[-hx,hy]])@np.array([[c,s],[-s,c]])+q[:2]
def polygon_distance(a,b):
 return min(min(float(ptseg(p,x,y)) for x,y in zip(b,np.roll(b,-1,axis=0))) for p in a)
def separation(q,lo,hi,extra):
 c,s=math.cos(q[2]),math.sin(q[2]);hx=.29+extra/2;hy=.155+extra/2;h=(hi-lo)/2+extra/2;v=(hi+lo)/2-q[:2]
 return [abs(v[0])-(hx*abs(c)+hy*abs(s)+h[0]),abs(v[1])-(hx*abs(s)+hy*abs(c)+h[1]),abs(v[0]*c+v[1]*s)-(hx+h[0]*abs(c)+h[1]*abs(s)),abs(-v[0]*s+v[1]*c)-(hy+h[0]*abs(s)+h[1]*abs(c))]
body=corners(q,.29,.155);b=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]]);gap=min(polygon_distance(body,b),polygon_distance(b,body));age=d['now']-o['stamp'];uncertainty=min(.55*age,.35*age+.04*age*age+.55*max(0,age-.35));sat=separation(q,lo,hi,extra)
result=dict(measured_returns=len(points),reference_centerline_to_measured_returns_m=center_distance(route),selected_path_centerline_to_measured_returns_m=center_distance(path),reference_swept_body_to_measured_returns_min_m=bodydistance(route),selected_path_swept_body_to_measured_returns_min_m=bodydistance(path),diagnostic_pose_body_to_measured_returns_m=float(distances(points,q).min()),diagnostic_pose_body_to_inflated_aabb_m=gap,diagnostic_sat_separations_m=sat,sat_reports_overlap=all(x<=0 for x in sat),effective_prediction_age_s=age,observation_age_s=o['observation_age_s'],extra_source_transport_allowance_s=.30,uncertainty_per_axis_m=uncertainty,measurement_padding_per_axis_m=.02,inflation_per_axis_m=uncertainty+.02,additional_clearance_m=extra,scope='Distances to recorded measured points, not a guarantee about unobserved material. Raw point-to-cluster labels absent: points selected inside exact recorded box on matching scan.',failure_chain='Extended returns -> axis-aligned box fills unobserved interior -> one-hit unknown-motion expansion -> 8 cm overlap buffer -> held path rejected -> bounded replacement search failed',runtime_changes=False)
assert result['sat_reports_overlap'];(R/'review/stop_20261116_155527/distance_analysis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
