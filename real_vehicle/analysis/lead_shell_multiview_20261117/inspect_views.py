"""Plot measured 2-D LiDAR views; no vehicle-control or recognition runtime changes."""
import json,math,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent;O=R/'inspection';O.mkdir(exist_ok=True)
views=[('rear',0,'Rear'),('rear_left',45,'Rear oblique A'),('left_side',90,'Side A'),('front_left',135,'Front oblique A'),('front',180,'Front'),('front_right',225,'Front oblique B'),('right_side',270,'Side B'),('rear_right',315,'Rear oblique B')]
def load(v):return [json.loads(s) for s in (R/v/'scans.jsonl').open()]
def matrix(rows):return np.array([[np.nan if v is None else v for v in row['ranges']] for row in rows])
bg=load('background');bm=np.nanmedian(matrix(bg),axis=0);base=bg[0];angles=base['angle_min']+np.arange(len(bm))*base['angle_increment'];validbg=np.isfinite(bm)&(bm>=base['range_min'])&(bm<10);bxy=np.column_stack((bm*np.cos(angles),bm*np.sin(angles)));tree=cKDTree(bxy[validbg]);report=[];sources={}
for name,angle,title in views:
 rows=load(name);meta=json.loads((R/name/'result.json').read_text());assert meta['passed'] and len(rows)==201
 m=matrix(rows);med=np.nanmedian(m,axis=0);valid=np.isfinite(med)&(med>=rows[0]['range_min'])&(med<=min(rows[0]['range_max'],10));xy=np.column_stack((med*np.cos(angles),med*np.sin(angles)));roi=valid&(xy[:,0]>=.7)&(xy[:,0]<=2.3)&(abs(xy[:,1])<=.65);fg=roi&(bm-med>.08);points=xy[fg];assert len(points)>=10
 # Same-beam comparison outside the object ROI screens F1/background changes.
 compare=valid&validbg&~roi;delta=abs(med[compare]-bm[compare]);matching=tree.query(xy[compare])[0]
 counts=[];chamfer=[]
 fg_tree=cKDTree(points)
 for row in m:
  p=np.column_stack((row*np.cos(angles),row*np.sin(angles)));sel=np.isfinite(row)&(row>=rows[0]['range_min'])&(row<10)&(p[:,0]>=.7)&(p[:,0]<=2.3)&(abs(p[:,1])<=.65)&(bm-row>.08)
  counts.append(int(sel.sum()))
  if sel.any():chamfer.append(float(np.median(fg_tree.query(p[sel])[0])))
 indices=np.flatnonzero(fg);groups=[]
 for k,point in zip(indices,points):
  if not groups or k-groups[-1][-1][0]>1 or np.linalg.norm(point-groups[-1][-1][1])>.08:groups.append([])
  groups[-1].append((int(k),point))
 groups=[g for g in groups if len(g)>=2]
 extent=np.ptp(points,axis=0);low=points.min(axis=0);high=points.max(axis=0)
 row=dict(view=name,operator_rotation_deg=angle,angle_is_approximate=True,scans=len(rows),duration_s=meta['duration_s'],median_scan_foreground_points=len(points),foreground_count_range=[min(counts),max(counts)],visible_bounds_laser_m=[*low,*high],visible_span_cm=list(extent*100),groups_with_at_least_two_points=len(groups),outside_roi_same_beam_median_m=float(np.median(delta)),outside_roi_same_beam_p90_m=float(np.percentile(delta,90)),outside_roi_nearest_background_p90_m=float(np.percentile(matching,90)),within_capture_median_point_to_template_p90_m=float(np.percentile(chamfer,90)),production_approved=False)
 # Registration is not assumed merely because the operator intended no motion.
 row['background_screen_passed']=row['outside_roi_same_beam_median_m']<.025 and row['outside_roi_nearest_background_p90_m']<.10
 payload=dict(metadata=row,laser_frame_xy_m=points.tolist(),centered_visible_xy_m=(points-points.mean(axis=0)).tolist(),notes=['2-D visible returns, not full 3-D body or full footprint.','Point centroid is not the physical vehicle centre.','Colour groups are scan segments, not labelled wheels.','Within-capture consistency is not independent recognition accuracy.'],production_approved=False)
 (O/(name+'_visible_template.json')).write_text(json.dumps(payload,indent=2)+'\n')
 fig,ax=plt.subplots(figsize=(8,6.5));fig.subplots_adjust(left=.13,right=.97,top=.88,bottom=.20)
 nearbg=bxy[validbg&(bxy[:,0]>=.8)&(bxy[:,0]<=2.3)&(abs(bxy[:,1])<=.7)]
 if len(nearbg):ax.scatter(nearbg[:,0],nearbg[:,1],marker='x',s=18,color='#b0b0b0',label='Empty-background returns')
 # All selected points are plotted; singletons remain visible in grey.
 ax.scatter(points[:,0],points[:,1],s=18,color='#555555',alpha=.6)
 colours=['#2166ac','#d07818','#7b3294','#16806a','#cc4c54','#5a638d']
 for j,g in enumerate(groups):
  a=np.array([z[1] for z in g]);ax.scatter(a[:,0],a[:,1],s=30,color=colours[j%len(colours)],label='Return group '+str(j+1))
 ax.set_xlim(.95,2.15);ax.set_ylim(-.6,.6);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set(xlabel='Laser-forward X (m)',ylabel='Laser-left Y (m)',title='New shell | '+title+' | approx. '+str(angle)+' deg')
 ax.legend(loc='upper left',fontsize=8)
 fig.text(.13,.09,str(len(points))+' median-scan foreground points | '+str(len(rows))+' scans / 5 s',fontsize=10)
 fig.text(.13,.05,'Actual 2-D LiDAR data. Segments are not labelled vehicle parts.',fontsize=9)
 fig.savefig(O/(name+'.png'),dpi=160);fig.savefig(O/(name+'.pdf'));plt.close(fig)
 sources[str(R/name/'scans.jsonl')]=hashlib.sha256((R/name/'scans.jsonl').read_bytes()).hexdigest();report.append(row)
summary=dict(capture_complete=True,views=report,total_target_scans=sum(z['scans'] for z in report),background_scans=len(bg),all_background_screens_passed=all(z['background_screen_passed'] for z in report),coordinate_frame='LiDAR: +X forward, +Y left',shell_version='User reports new shell installed',dimensions_surveyed=False,target_presence='Operator placement plus difference from initial empty background',independent_recognition_validated=False,recognizer_installed=False,runtime_changed=False,actuator_nodes_started=0)
(O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(O/'sources.json').write_text(json.dumps(sources,indent=2)+'\n');(R/'collection_manifest.json').write_text(json.dumps(dict(captured_views=[x[0] for x in views],background='background',source_data_retained=True,summary='inspection/summary.json',reference_templates_production_approved=False,runtime_changed=False),indent=2)+'\n')
print(json.dumps(dict(total_scans=summary['total_target_scans'],all_background_screens_passed=summary['all_background_screens_passed'],views=[{k:z[k] for k in ['view','median_scan_foreground_points','visible_span_cm','groups_with_at_least_two_points','outside_roi_nearest_background_p90_m','background_screen_passed']} for z in report]),indent=2))
