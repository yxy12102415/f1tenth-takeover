from pathlib import Path
import json,math,yaml,numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon
R=Path(__file__).resolve().parents[1];run=R/'run_logs/20261115_225052-closed-frenet-ground-attempt';trace=json.loads((run/'tracking_trace.json').read_text());cmd=json.loads((run/'command_trace.json').read_text());assets=json.loads((R/'runtime_assets.json').read_text())
start=cmd['rpm_command'][0][0];bad=next(v for v in trace['controller'] if v['received_mono']>=start and not v.get('valid'));stop=bad['received_mono'];poses=[p for p in trace['poses'] if p['received_mono']>=start-.1];before=[p for p in poses if p['received_mono']<=stop];after=[before[-1]]+[p for p in poses if p['received_mono']>stop]
route=np.array(assets['route']);selected=None
for row in trace['controller']:
 if row.get('plan_id')==bad['plan_id'] and row.get('local_path_snapshot'):selected=np.array(row['local_path_snapshot']);break
initial=next(v for v in reversed(trace['controller']) if v['received_mono']<start and v.get('valid') and v.get('occupancies'))
obstacle=next(o for o in initial['occupancies'] if 2.7<.5*(o['xmin']+o['xmax'])<3.3 and abs(.5*(o['ymin']+o['ymax']))<.4)
meta=yaml.safe_load((R.parents[1]/'maps/381_2_0.yaml').read_text());impath=Path(meta['image']);impath=impath if impath.is_absolute() else R.parents[1]/'maps'/impath;image=Image.open(impath);ox,oy=meta['origin'][:2];res=meta['resolution'];w,h=image.size
fig,ax=plt.subplots(figsize=(12,6.6));ax.imshow(image,cmap='gray',vmin=0,vmax=255,extent=(ox,ox+w*res,oy,oy+h*res),origin='upper',alpha=.7)
ax.plot(route[:,0],route[:,1],'--',color='#666666',lw=2,label='Reference route',zorder=3)
if selected is not None:ax.plot(selected[:,0],selected[:,1],color='#dc9516',lw=2,label='Selected local path before STOP',zorder=4)
ax.plot([p['x'] for p in before],[p['y'] for p in before],color='#1167b1',lw=3,label='Driven trajectory (PF estimate)',zorder=6)
ax.plot([p['x'] for p in after],[p['y'] for p in after],color='#c7473c',lw=3,ls=':',label='Motion after STOP command',zorder=6)
s=before[-1];last=poses[-1];ax.scatter([s['x']],[s['y']],marker='X',s=90,color='#1167b1',zorder=9);ax.annotate('STOP command',(s['x'],s['y']),xytext=(.4,-.83),arrowprops=dict(arrowstyle='->',color='#1167b1'),color='#1167b1',fontsize=10)
ax.scatter([last['x']],[last['y']],s=60,color='#c7473c',zorder=9);ax.annotate('Rest position',(last['x'],last['y']),xytext=(1.18,-.85),arrowprops=dict(arrowstyle='->',color='#c7473c'),color='#c7473c',fontsize=10)
for p,color in [(s,'#1167b1'),(last,'#c7473c')]:
 c,ss=math.cos(p['yaw']),math.sin(p['yaw']);corners=np.array([[-.29,-.155],[.29,-.155],[.29,.155],[-.29,.155]])@np.array([[c,ss],[-ss,c]])+np.array([p['x'],p['y']]);ax.add_patch(Polygon(corners,closed=True,fill=False,ec=color,lw=1.2,zorder=5))
x,y=obstacle['xmin'],obstacle['ymin'];ax.add_patch(Rectangle((x,y),obstacle['xmax']-x,obstacle['ymax']-y,fc='#3a9d5d',ec='#176a32',alpha=.6,label='Observed obstacle cluster',zorder=7));ax.annotate('Observed obstacle',(x+.1,y+.15),xytext=(2.65,.65),arrowprops=dict(arrowstyle='->',color='#176a32'),color='#176a32',fontsize=10)
ax.scatter([1.993],[-1.057],marker='x',color='#a12b93',s=70,label='Map-adjacent return (replay conflict)',zorder=8)
ax.annotate('Single-hit wall-adjacent return',(1.993,-1.057),xytext=(2.5,-1.35),arrowprops=dict(arrowstyle='->',color='#a12b93'),color='#a12b93',fontsize=9)
ax.scatter(route[0,0],route[0,1],s=45,color='black',zorder=9);ax.text(route[0,0]-.25,route[0,1]+.15,'Start',fontsize=9);ax.scatter(route[-1,0],route[-1,1],marker='s',s=40,color='black',zorder=9);ax.text(route[-1,0]+.08,route[-1,1],'Goal',fontsize=9)
ax.set_xlim(-.6,6.1);ax.set_ylim(-1.65,1.8);ax.set_aspect('equal');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)');ax.set_title('381 | PID ground test: STOP before reaching the obstacle');ax.legend(loc='upper left',fontsize=8,framealpha=.95);ax.grid(alpha=.15)
fig.text(.12,.035,'PF-estimated trajectory. Green box is measured LiDAR support, not a surveyed vehicle footprint.',fontsize=9,color='#555555');fig.subplots_adjust(left=.08,right=.98,bottom=.12,top=.94)
fig.savefig(R/'review/ground_20261115_225052_path.png',dpi=180);fig.savefig(R/'review/ground_20261115_225052_path.pdf');plt.close(fig)
def cross(p):
 best=(float('inf'),0)
 for a,b in zip(route[:-1,:2],route[1:,:2]):
  e=b-a;u=np.clip((np.array([p['x'],p['y']])-a).dot(e)/e.dot(e),0,1);d=np.array([p['x'],p['y']])-(a+u*e);norm=np.linalg.norm(d)
  if norm<best[0]:best=(norm,(e[0]*d[1]-e[1]*d[0])/np.linalg.norm(e))
 return best[1]
summary=dict(time_to_stop_s=stop-start,estimated_stop_command_pose=[s[k] for k in ['x','y','yaw']],estimated_rest_pose=[last[k] for k in ['x','y','yaw']],max_cross_track_before_stop_m=max(abs(cross(p)) for p in before),rest_cross_track_m=cross(last),estimated_coast_displacement_m=math.hypot(last['x']-s['x'],last['y']-s['y']),observed_obstacle_bbox={k:obstacle[k] for k in ['xmin','xmax','ymin','ymax']},local_path_snapshot_found=selected is not None,exact_collision_guard_observation_was_logged=False)
(R/'review/ground_20261115_225052_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
