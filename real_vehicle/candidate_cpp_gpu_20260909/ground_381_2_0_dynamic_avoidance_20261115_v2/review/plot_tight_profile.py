from pathlib import Path
import json,math,yaml,numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
R=Path(__file__).resolve().parents[1];data=json.loads((R/'review/tight_profile_replay.json').read_text());assets=json.loads((R/'runtime_assets.json').read_text());route=np.array(assets['route']);old=np.array(data['old']['path']);new=np.array(data['new']['path'])
meta=yaml.safe_load((R.parents[1]/'maps/381_2_0.yaml').read_text());p=Path(meta['image']);p=p if p.is_absolute() else R.parents[1]/'maps'/p;im=Image.open(p);ox,oy=meta['origin'][:2];res=meta['resolution'];w,h=im.size
fig,ax=plt.subplots(figsize=(12,6));ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=(ox,ox+w*res,oy,oy+h*res),origin='upper',alpha=.7)
ax.plot(route[:,0],route[:,1],'--',color='#666666',lw=2,label='Reference route');ax.plot(old[:,0],old[:,1],color='#de8f24',lw=2,label='Previous profile: 14 cm + 7 cm reserve');ax.plot(new[:,0],new[:,1],color='#097a9c',lw=2.5,label='Tight profile: 10 cm + 2 cm reserve')
trace=json.loads((R/'run_logs/20261115_233514-closed-frenet-ground-attempt/tracking_trace.json').read_text());row=next(v for v in reversed(trace['controller']) if v.get('valid') and v.get('occupancies') and v.get('scan_pose'));b=next(o for o in row['occupancies'] if not o['map_supported'] and 2.5<(o['xmin']+o['xmax'])/2<3.5 and abs((o['ymin']+o['ymax'])/2)<.5);ax.add_patch(Rectangle((b['xmin'],b['ymin']),b['xmax']-b['xmin'],b['ymax']-b['ymin'],facecolor='#32a05a',edgecolor='#176a32',alpha=.6,label='Observed obstacle support'))
ax.scatter(route[0,0],route[0,1],c='k');ax.text(route[0,0]-.1,route[0,1]+.13,'Start');ax.scatter(route[-1,0],route[-1,1],c='k',marker='s');ax.text(route[-1,0]+.08,route[-1,1],'Goal')
ax.set_xlim(-.5,5.9);ax.set_ylim(-1.4,1.5);ax.set_aspect('equal');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)');ax.set_title('381 | Tighter passing profile | captured-scene replay');ax.legend(loc='upper left',fontsize=9);ax.grid(alpha=.15);fig.text(.11,.025,'Captured initial scene replay. Planned geometry only; the revised path has not been driven.',fontsize=9,color='#555555');fig.subplots_adjust(bottom=.14,top=.93,left=.08,right=.98);fig.savefig(R/'review/tight_profile_comparison.png',dpi=180)
# Quantify early deviation relative to the original route using matching base indices.
assert len(old)==len(new)
# base begins at the route index nearest to the old path start.
i=int(np.argmin(np.linalg.norm(route[:,:2]-old[0,:2],axis=1)));base=route[i:i+len(old)];arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1))];normal=np.c_[-np.sin(base[:,2]),np.cos(base[:,2])]
def early(path):
 d=np.sum((path[:,:2]-base[:,:2])*normal,axis=1);return float(abs(np.interp(1.,arc,d)))
report=dict(old_deviation_at_1m_m=early(old),new_deviation_at_1m_m=early(new),departure_s_m=data['new']['score_terms'].get('departure_s',0.),clearance_m=data['new']['score_terms']['generic_required_body_gap_m'],same_goal=bool(np.linalg.norm(old[-1,:2]-new[-1,:2])<1e-9),new_maximum_curvature_1pm=data['new']['maximum_curvature_1pm']);assert report['same_goal'];(R/'review/tight_profile_geometry_comparison.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
