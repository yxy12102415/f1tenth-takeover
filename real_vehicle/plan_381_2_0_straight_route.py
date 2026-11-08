#!/usr/bin/env python3
"""Offline straight-route construction from the immutable tested start pose."""
import csv,json,math,hashlib
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
WS=Path(__file__).resolve().parent
SOURCE=WS/'routes/381_2_0_original_start_extended_end_20261107_v1.csv'
OUT=WS/'routes/381_2_0_straight_original_start_20261107_v1.csv'
REPORT=OUT.with_suffix('.geometry.json')
PLOT=WS/'maps/381_2_0_straight_original_start_20261107_v1.png'

def build():
 if OUT.exists() or REPORT.exists():raise RuntimeError('Version exists; preserve prior route evidence')
 original=np.loadtxt(SOURCE,delimiter=',',skiprows=1);start=original[0,:3].copy()
 meta=yaml.safe_load((WS/'maps/381_2_0.yaml').read_text());im=np.asarray(Image.open(meta['image']));res=float(meta['resolution']);origin=np.array(meta['origin'][:2]);free=(im>=250)&((1.-im/255.)<meta['free_thresh'])
 field=distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*res
 def clearance(p):
  q=np.floor((p-origin)/res).astype(int);row=im.shape[0]-1-q[...,1];valid=(q[...,0]>=0)&(q[...,0]<im.shape[1])&(row>=0)&(row<im.shape[0])
  value=field[np.clip(row,0,im.shape[0]-1),np.clip(q[...,0],0,im.shape[1]-1)]
  value-=np.linalg.norm(p-(origin+(q+.5)*res),axis=-1)+res/math.sqrt(2)
  return np.where(valid,np.maximum(0,value),0.)
 s=np.arange(0,9.001,.01);tangent=np.array([math.cos(start[2]),math.sin(start[2])]);xy=start[:2]+s[:,None]*tangent
 us=np.linspace(-.29,.29,25);vs=np.linspace(-.155,.155,15);offset=np.array([(u,v) for u in us for v in vs]);h=start[2];rot=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
 sampled_body=clearance(xy[:,None,:]+offset@rot.T).min(axis=1)
 # Clearance is 1-Lipschitz: account for gaps between body samples and poses.
 coverage=.5*math.hypot(us[1]-us[0],vs[1]-vs[0])+.005
 body=np.maximum(0,sampled_body-coverage);center=np.maximum(0,clearance(xy)-.005)
 bad=np.flatnonzero((body<.15)|(center<.35));limit=float(s[bad[0]-1])
 # Reserve >=0.5 m beyond the designated endpoint inside the screened corridor.
 length=math.floor((limit-.50)*10)/10
 assert length>=5.0
 distance=np.linspace(0,length,int(round(length/.05))+1);points=start[:2]+distance[:,None]*tangent
 route=np.c_[points,np.full(len(distance),start[2])];route[0]=start
 mask=s<=length+1e-9
 report=dict(map='381_2_0',route=str(OUT),source=str(SOURCE),geometry='single_straight',route_mode='open_stop_at_end',length_m=length,start_pose=start.tolist(),end_pose=route[-1].tolist(),start_position_error_m=float(np.linalg.norm(route[0,:2]-start[:2])),start_heading_error_rad=float(route[0,2]-start[2]),maximum_curvature_1pm=0.,required_steering_rad=0.,spacing_m=.05,unknown_blocked=True,assumed_body_length_width_m=[.58,.31],body_dimensions_confirmed=False,minimum_body_clearance_lower_bound_m=float(body[mask].min()),minimum_center_clearance_lower_bound_m=float(center[mask].min()),endpoint_body_clearance_lower_bound_m=float(body[int(round(length/.01))]),forward_corridor_clearance_gate_m=.15,forward_corridor_end_distance_m=limit,forward_corridor_beyond_endpoint_m=limit-length,coverage_correction_m=coverage,map_resolution_m=res,static_screen_passed=bool(body[mask].min()>=.30),ground_test_passed=False,overtaking_feasibility_validated=False,scope='Static saved-map corridor screen for assumed rectangle; does not establish live localization, braking performance or two-car passing space.',map_sha256=hashlib.sha256(Path(meta['image']).read_bytes()).hexdigest())
 assert report['static_screen_passed']
 with OUT.open('w') as f:
  writer=csv.writer(f);writer.writerow(['x','y','yaw']);writer.writerows(route)
 REPORT.write_text(json.dumps(report,indent=2)+'\n')
 fig,ax=plt.subplots(figsize=(11,8));extent=[origin[0],origin[0]+im.shape[1]*res,origin[1],origin[1]+im.shape[0]*res]
 ax.imshow(im,cmap='gray',origin='upper',extent=extent,vmin=0,vmax=255)
 ax.plot(original[:,0],original[:,1],color='#8293a3',lw=1.8,ls='--',label='Previous route: 6.74 m')
 ax.plot(route[:,0],route[:,1],color='#087fce',lw=3,label=f'New straight route: {length:.2f} m')
 forward=np.array([route[-1,:2],start[:2]+limit*tangent]);ax.plot(forward[:,0],forward[:,1],color='#cc842b',lw=2,ls=':',label=f'Additional map corridor: {limit-length:.2f} m')
 for p,label,col,off in [(route[0],'Fixed start','#16964c',(-10,35)),(route[-1],'New end','#c4432e',(-15,-45))]:
  ax.scatter(p[0],p[1],s=85,c=col,zorder=6);ax.annotate(f'{label}\n({p[0]:.2f}, {p[1]:.2f})',p[:2],xytext=off,textcoords='offset points',ha='center',bbox=dict(facecolor='white',alpha=.9,edgecolor='none'),arrowprops=dict(arrowstyle='-',color=col))
  corners=np.array([[-.29,-.155],[.29,-.155],[.29,.155],[-.29,.155],[-.29,-.155]])@rot.T+p[:2];ax.plot(corners[:,0],corners[:,1],color=col,lw=1)
 ax.annotate('',xy=route[65,:2],xytext=route[50,:2],arrowprops=dict(arrowstyle='->',color='#087fce',lw=3))
 ax.set(title='381 2.0 | Straight route, same start and heading',xlabel='Map X [m]',ylabel='Map Y [m]',xlim=(-1.2,7.5),ylim=(-2.7,3.5));ax.set_aspect('equal');ax.legend(loc='upper left');ax.text(.02,.02,'Saved-map planning only | Vehicle body assumed 0.58 x 0.31 m\nNew route has not been driven; overtaking space is not yet validated.',transform=ax.transAxes,fontsize=9,bbox=dict(facecolor='white',alpha=.92,edgecolor='none'));fig.tight_layout();fig.savefig(PLOT,dpi=160);fig.savefig(PLOT.with_suffix('.pdf'));plt.close(fig)
 print(json.dumps(report,indent=2))
if __name__=='__main__':build()
