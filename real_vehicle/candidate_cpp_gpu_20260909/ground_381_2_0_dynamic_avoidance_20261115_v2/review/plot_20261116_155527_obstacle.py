from pathlib import Path
import json,math,shutil
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon
R=Path(__file__).resolve().parents[1];D=R/'review/stop_20261116_155527';D.mkdir(exist_ok=True)
run=R/'run_logs/20261116_155527-closed-frenet-ground-attempt'
trace=json.loads((run/'tracking_trace.json').read_text());cmd=json.loads((run/'command_trace.json').read_text());assets=json.loads((R/'runtime_assets.json').read_text());bad=json.loads((R/'review/dmpc_20261116_155527_moving_failure.json').read_text());diag=bad['candidate_collision_diagnostics'][0];box=diag['occupancy'];source=next(x for x in reversed(trace['controller']) if x.get('valid') and x.get('input_scan_stamp')==box['stamp'] and any(o.get('id')==box['id'] for o in x.get('occupancies',[])))
points=np.array(source['obstacle_points']);static=np.array(source['static_obstacle_points']);inside=(points[:,0]>=box['xmin']-1e-6)&(points[:,0]<=box['xmax']+1e-6)&(points[:,1]>=box['ymin']-1e-6)&(points[:,1]<=box['ymax']+1e-6);highlight=points[inside]
path=np.array(next(x['local_path_snapshot'] for x in trace['controller'] if x.get('plan_id')==source['plan_id'] and x.get('local_path_snapshot')));route=np.array(assets['route']);start=next(x['receipt_mono'] for x in cmd['health'] if abs(x['erpm'])>100);stop=bad['received_mono'];poses=[x for x in trace['poses'] if x['received_mono']>=start-.15];before=[x for x in poses if x['received_mono']<=stop];after=[before[-1]]+[x for x in poses if x['received_mono']>stop];rest=poses[-1];stopped=before[-1];conflict=diag['ego_pose']
meta_path=next(Path(p) for p in assets['sources'] if p.endswith('.yaml'));meta=yaml.safe_load(meta_path.read_text());impath=Path(meta['image']);impath=impath if impath.is_absolute() else meta_path.parent/impath;im=Image.open(impath);ox,oy=meta['origin'][:2];res=meta['resolution'];w,h=im.size
near=next(o for o in source['occupancies'] if o['id']==10)
def body(ax,p,color,**kw):
 x,y,yaw=(p['x'],p['y'],p['yaw']) if isinstance(p,dict) else p;c,s=math.cos(yaw),math.sin(yaw);vertices=np.array([[-.29,-.155],[.29,-.155],[.29,.155],[-.29,.155]])@np.array([[c,s],[-s,c]])+[x,y];ax.add_patch(Polygon(vertices,closed=True,fill=False,ec=color,lw=1.8,zorder=8,**kw))
def draw(zoom=False):
 fig,ax=plt.subplots(figsize=(13,8) if zoom else (14,7.8));ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=(ox,ox+w*res,oy,oy+h*res),origin='upper',alpha=.65)
 ax.scatter(static[:,0],static[:,1],s=5,c='#8c8c8c',alpha=.45,label='Other LiDAR returns',zorder=2)
 ax.scatter(points[~inside,0],points[~inside,1],s=6,c='#75839a',alpha=.55,zorder=3)
 ax.plot(route[:,0],route[:,1],'--',c='#505050',lw=2,label='Reference route',zorder=4)
 ax.plot(path[:,0],path[:,1],c='#d6a11b',lw=2.2,label='Last accepted local path',zorder=5)
 ax.plot([p['x'] for p in before],[p['y'] for p in before],c='#006bb6',lw=3,label='Driven trajectory (PF estimate)',zorder=6)
 ax.plot([p['x'] for p in after],[p['y'] for p in after],':',c='#006bb6',lw=2.8,label='Motion after STOP command',zorder=6)
 ax.scatter(highlight[:,0],highlight[:,1],s=15,c='#c91c2d',label='Measured returns inside red box',zorder=9)
 ax.add_patch(Rectangle((box['xmin'],box['ymin']),box['xmax']-box['xmin'],box['ymax']-box['ymin'],fill=False,ec='#bc1629',lw=2,label='Measured-support bounding box',zorder=7))
 lo=diag['predicted_box'];ax.add_patch(Rectangle((lo[0],lo[1]),lo[2]-lo[0],lo[3]-lo[1],facecolor='#db3145',alpha=.08,zorder=3));ax.add_patch(Rectangle((lo[0],lo[1]),lo[2]-lo[0],lo[3]-lo[1],fill=False,ec='#db3145',ls='--',lw=1.8,label='Inflated planning envelope',zorder=7))
 ax.add_patch(Rectangle((near['xmin'],near['ymin']),near['xmax']-near['xmin'],near['ymax']-near['ymin'],fill=False,ec='#24804a',lw=2,label='Near-route obstacle cluster',zorder=8))
 body(ax,conflict,'#ac2134');ax.scatter([conflict[0]],[conflict[1]],marker='x',s=75,c='#ac2134',zorder=10)
 ax.scatter(route[0,0],route[0,1],s=40,c='k',zorder=10);ax.scatter(route[-1,0],route[-1,1],s=50,c='k',marker='s',zorder=10)
 ax.annotate('Goal',route[-1,:2],xytext=(5.68,1.24),arrowprops=dict(arrowstyle='->',color='#444444'),fontsize=10)
 if zoom:
  ax.annotate('Measured points form an extended contour\nBox interior is not measured solid material',(5.83,-.48),xytext=(4.0,-1.94),arrowprops=dict(arrowstyle='->',color='#bc1629'),color='#a41728',fontsize=11)
  ax.annotate('Predicted conflict pose\n(not actual F1 position)',conflict[:2],xytext=(3.94,.64),arrowprops=dict(arrowstyle='->',color='#a41728'),color='#a41728',fontsize=10)
  ax.set_xlim(3.85,6.85);ax.set_ylim(-2.15,1.7);ax.set_title('STOP trigger | Route-end LiDAR returns and planning envelope',fontsize=14)
 else:
  body(ax,stopped,'#006bb6');ax.annotate('STOP command\nF1 was here',(stopped['x'],stopped['y']),xytext=(.1,-1.32),arrowprops=dict(arrowstyle='->',color='#006bb6'),color='#006bb6',fontsize=11)
  ax.annotate('Near-route cluster\n(check against opponent position)',((near['xmin']+near['xmax'])/2,near['ymax']),xytext=(2.0,.61),arrowprops=dict(arrowstyle='->',color='#24804a'),color='#24804a',fontsize=10)
  ax.annotate('Trigger region: far end of route',(5.8,-.6),xytext=(3.7,-1.95),arrowprops=dict(arrowstyle='->',color='#a41728'),color='#a41728',fontsize=11)
  ax.annotate('Future path conflict\nF1 did not reach this point',conflict[:2],xytext=(3.6,.65),arrowprops=dict(arrowstyle='->',color='#a41728'),color='#a41728',fontsize=10)
  ax.text(route[0,0]-.05,route[0,1]+.15,'Start',fontsize=10);ax.set_xlim(-.65,6.95);ax.set_ylim(-2.1,2.05);ax.set_title('381 | Where the planner rejected the path',fontsize=16)
 ax.set_aspect('equal');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)');ax.grid(alpha=.12);ax.legend(loc='upper left',bbox_to_anchor=(1.02,1.) if zoom else (0.,1.),fontsize=8,framealpha=.96)
 fig.text(.08,.035,'Recorded LiDAR points + saved map. Boxes are algorithmic regions, not verified object dimensions.\nHighlighted points share the box region; point-to-cluster membership was not recorded.',fontsize=9,color='#555555');fig.subplots_adjust(bottom=.13,left=.08,right=.70 if zoom else .98,top=.94)
 name='obstacle_zoom' if zoom else 'obstacle_location';fig.savefig(D/(name+'.png'),dpi=170);fig.savefig(D/(name+'.pdf'));plt.close(fig)
draw();draw(True)
summary=dict(run=str(run),source_plan_id=source['plan_id'],source_scan_stamp=source['input_scan_stamp'],trigger_occupancy_id=box['id'],highlighted_measured_returns=len(highlight),source_scan_matches_box_stamp=source['input_scan_stamp']==box['stamp'],box_bounds={k:box[k] for k in ['xmin','ymin','xmax','ymax']},expanded_bounds=diag['predicted_box'],planning_pose=bad['planning_pose'],stop_command_estimated_pose=[stopped[k] for k in ['x','y','yaw']],rest_estimated_pose=[rest[k] for k in ['x','y','yaw']],predicted_conflict_pose=conflict,raw_cluster_membership_available=False,warning='The red raw box encloses an extended set of LiDAR returns; its interior is not necessarily occupied. Physical object identity is not established.')
(D/'provenance.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
shutil.copy2(D/'obstacle_location.png','/home/f1tenth/381_obstacle_location.png');shutil.copy2(D/'obstacle_zoom.png','/home/f1tenth/381_obstacle_zoom.png')
