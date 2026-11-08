#!/usr/bin/env python3
"""Optimize a longer open route with the commissioned start pose held fixed."""
import json,csv,math
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import map_coordinates, distance_transform_edt
from scipy.optimize import minimize
D=Path(__file__).resolve().parent
A=json.loads((D/'assets/381_2_0.json').read_text());start=np.array(A['route'][0]);old=np.array(A['route']);res=A['resolution'];origin=np.array(A['origin']);grid=np.array(A['clearance']).reshape(A['height'],A['width']);grid=grid-distance_transform_edt(grid<=0)*res
def clearance(p):
 p=np.asarray(p);q=(p-origin)/res
 return map_coordinates(grid,[A['height']-1-q[...,1],q[...,0]],order=1,mode='constant',cval=0.)-.5*res*math.sqrt(2)
seed=np.array([start[:2],[1.8,-.02],[3.8,.15],[5.5,.45],[5.6,-.7],[4.3,-2.0],[3.2,-3.0],[1.7,-2.1],[.5,-1.0],[.10,-.50]])
t=np.r_[0,np.cumsum(np.linalg.norm(np.diff(seed,axis=0),axis=1))];sample=np.linspace(0,t[-1],220)
def evaluate(z,ts=sample):
 pts=seed.copy();pts[1:-1]=z.reshape(-1,2)
 spline=CubicSpline(t,pts,bc_type=((1,[math.cos(start[2]),math.sin(start[2])]),(1,[-.35,.93675])))
 xy=spline(ts);v=spline(ts,1);acc=spline(ts,2);norm=np.linalg.norm(v,axis=1);tangent=v/np.maximum(norm[:,None],1e-8);normal=np.c_[-tangent[:,1],tangent[:,0]];k=(v[:,0]*acc[:,1]-v[:,1]*acc[:,0])/np.maximum(norm**3,1e-8)
 bodies=[]
 for u in np.linspace(-.29,.29,7):
  for w in [-.155,.155]:bodies.append(clearance(xy+u*tangent+w*normal))
 for u in [-.29,.29]:
  for w in [-.10,0,.10]:bodies.append(clearance(xy+u*tangent+w*normal))
 return xy,norm,k,clearance(xy),np.array(bodies).min(axis=0),spline
z0=seed[1:-1].ravel()
def constraints(z):
 xy,v,k,c,b,_=evaluate(z)
 return np.r_[c-.30,b-.06,1.03-np.abs(k),v-.22]
def objective(z):
 xy,v,k,c,b,_=evaluate(z)
 return .15*np.sum((z-z0)**2)+.015*np.sum(k*k)+.005*np.sum(np.diff(k)**2)-.02*np.sum(np.linalg.norm(np.diff(xy,axis=0),axis=1))
bounds=[(v-1.3,v+1.3) for v in z0]
warm=z0.copy()
previous=D/'reports/long_route_geometry.json'
if previous.exists():warm=np.array(json.loads(previous.read_text())['control_points'])[1:-1].ravel()
penalty=lambda z: objective(z)+1500*np.sum(np.minimum(constraints(z),0.)**2)
pre=minimize(penalty,warm,method='L-BFGS-B',bounds=bounds,options={'maxiter':240,'maxfun':12000,'ftol':1e-10})
result=minimize(objective,pre.x,method='SLSQP',bounds=bounds,constraints={'type':'ineq','fun':constraints},options={'maxiter':300,'ftol':1e-7})
xy,v,k,c,b,spline=evaluate(result.x,np.linspace(0,t[-1],2001));arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
ss=np.linspace(0,arc[-1],int(np.ceil(arc[-1]/.04))+1);tt=np.interp(ss,arc,np.linspace(0,t[-1],2001));xy,v,k,c,b,spline=evaluate(result.x,tt);der=spline(tt,1);yaw=np.arctan2(der[:,1],der[:,0]);ss=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
passed=bool(c.min()>=.295 and b.min()>=.055 and abs(k).max()<=1.04 and v.min()>.2)
report={'candidate_geometry_passed':passed,'physical_motion_ready':False,'optimizer_message':str(result.message),'length_m':float(ss[-1]),'start_position_error_m':float(np.linalg.norm(xy[0]-start[:2])),'start_heading_error_rad':float(abs(np.angle(np.exp(1j*(yaw[0]-start[2]))))),'minimum_center_clearance_conservative_m':float(c.min()),'minimum_sampled_body_clearance_conservative_m':float(b.min()),'maximum_curvature_1pm':float(abs(k).max()),'maximum_required_steering_rad':float(np.arctan(.25*abs(k)).max()),'assumed_body_m':[.58,.31],'scope':'Dense static geometry screening only. Time-indexed overtaking and real stop/guard acceptance pending.','control_points':np.vstack([seed[0],result.x.reshape(-1,2),seed[-1]]).tolist()}
(D/'reports/long_route_geometry.json').write_text(json.dumps(report,indent=2)+'\n')
if passed:
 speeds=np.minimum(1.5,np.sqrt(1.2/np.maximum(abs(k),.01)));speeds=np.maximum(.7,speeds);speeds[0]=speeds[-1]=0.
 with (D/'assets/381_2_0_overtake_candidate.csv').open('w') as f:
  writer=csv.writer(f);writer.writerow(['x','y','yaw','distance_m','curvature_1pm','speed_mps']);writer.writerows(np.c_[xy,yaw,ss,k,speeds])
 # This is a cruise reference; endpoints 0, terminal policy still pending admission.
 data=dict(A);data.update(route_mode='open_stop_at_end',route=np.c_[xy,yaw].tolist(),speed=speeds.tolist(),preparation_only=True)
 (D/'assets/381_2_0_overtake_candidate.json').write_text(json.dumps(data,separators=(',',':'))+'\n')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
im=np.array(Image.open('/home/f1tenth/f1tenth_real_migration_ws_20260829/maps/381_2_0.pgm'))
fig,ax=plt.subplots(figsize=(10,8));ax.imshow(im,cmap='gray',origin='upper',extent=[origin[0],origin[0]+A['width']*res,origin[1],origin[1]+A['height']*res],vmin=0,vmax=255);ax.plot(old[:,0],old[:,1],'--',color='gray',label='Previous route: 5.85 m');ax.plot(xy[:,0],xy[:,1],color='#2877c5',lw=2,label='Candidate: %.2f m'%ss[-1]);ax.scatter(*xy[0],c='green',s=60,label='Fixed start');ax.scatter(*xy[-1],c='red',s=60,label='End');
for distance in np.arange(2,ss[-1],2):
 i=np.argmin(abs(ss-distance));ax.text(xy[i,0]+.1,xy[i,1]+.1,str(int(distance))+' m',fontsize=8)
ax.set(xlabel='Map X [m]',ylabel='Map Y [m]',title='381 2.0 — Longer Route Candidate (Not Motion Approved)');ax.legend(fontsize=9);ax.set_aspect('equal');fig.tight_layout();fig.savefig(D/'reports/381_long_route_en.png',dpi=160)
print(json.dumps(report,indent=2))
