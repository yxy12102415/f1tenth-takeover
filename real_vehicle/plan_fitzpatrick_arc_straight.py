#!/usr/bin/env python3
"""Map-constrained analytic arcs/straights with short linear-curvature blends.

Offline geometry only. Exact fixed start, pose closure and periodic curvature.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from numpy.polynomial.legendre import leggauss
from scipy.optimize import minimize
from scipy.spatial import cKDTree

WS=Path(__file__).resolve().parent
sys.path.insert(0,str(WS/'src/f1tenth_safety_migration/f1tenth_safety_migration'))
from route_preprocessor import _map_distance_field, _field_clearance, map_hazards

QX,QW=leggauss(16)
TRANSITION=.60


def displacement(s,heading,k0,k1,length):
    """Integrate a segment from its origin, independently at every s."""
    s=np.atleast_1d(s).astype(float)
    if abs(k1-k0)<1e-14:
        if abs(k0)<1e-14:
            return s[:,None]*np.array([math.cos(heading),math.sin(heading)])
        return np.column_stack(((np.sin(heading+k0*s)-math.sin(heading))/k0,
                                (math.cos(heading)-np.cos(heading+k0*s))/k0))
    t=s[:,None]*(QX[None,:]+1)/2
    yaw=heading+k0*t+.5*(k1-k0)/length*t*t
    return np.column_stack((np.cos(yaw)@QW,np.sin(yaw)@QW))*s[:,None]/2


def turn_segments(r0,r1,r2,r3):
    t=TRANSITION;k0=1/r0
    head=[('initial_long_arc',math.pi*r0/2-t/2,k0,k0),
          ('head_to_straight',t,k0,0.)]
    def corner(name,r):
        return [(name+'_entry',t,0.,1/r),(name+'_arc',math.pi*r/2-t,1/r,1/r),
                (name+'_exit',t,1/r,0.)]
    final=[('bottom_left_entry',t,0.,1/r3),
           ('bottom_left_arc',math.pi*r3/2-t/2-t/2*(1+r3/r0),1/r3,1/r3),
           ('return_to_initial_curvature',t,1/r3,k0)]
    return head,corner('top_right',r1),corner('top_left',r2),final


def endpoint(segments,heading):
    p=np.zeros(2)
    for _,length,k0,k1 in segments:
        p+=displacement([length],heading,k0,k1,length)[0]
        heading+=(k0+k1)*length/2
    return p


def layout(params):
    r0,r1,r2,r3,right_length=params
    turns=turn_segments(r0,r1,r2,r3)
    total=sum((endpoint(turn,i*math.pi/2) for i,turn in enumerate(turns)),np.zeros(2))
    top_length=float(total[0]);left_length=float(right_length+total[1])
    segments=(turns[0]+[('right_straight',right_length,0.,0.)]+turns[1]
        +[('top_straight',top_length,0.,0.)]+turns[2]
        +[('left_straight',left_length,0.,0.)]+turns[3])
    return segments,(right_length,top_length,left_length)


def evaluate(params,start,heading,step=.05,samples_per_segment=None):
    segments,lines=layout(params)
    if min(s[1] for s in segments)<=0:
        raise ValueError('Nonpositive segment length')
    rows=[];definitions=[];p=start.copy();yaw=heading;s0=0.
    for index,(name,length,k0,k1) in enumerate(segments):
        count=(max(2,int(math.ceil(length/step))) if samples_per_segment is None
               else samples_per_segment)
        local_s=np.linspace(0,length,count+1)
        xy=p+displacement(local_s,yaw,k0,k1,length)
        theta=yaw+k0*local_s+.5*(k1-k0)/length*local_s**2
        curvature=k0+(k1-k0)*local_s/length
        block=np.column_stack([s0+local_s,xy,theta,curvature,np.full(len(local_s),index)])
        rows.append(block if index==0 else block[1:])
        kind='transition' if abs(k0-k1)>1e-14 else ('straight' if k0==0 else 'arc')
        definitions.append(dict(name=name,kind=kind,start_s_m=s0,length_m=length,
            curvature_start_1pm=k0,curvature_end_1pm=k1,
            radius_m=None if kind!='arc' else 1/k0,
            start_pose=[float(p[0]),float(p[1]),float(yaw)],
            end_pose=[float(xy[-1,0]),float(xy[-1,1]),float(theta[-1])]))
        p=xy[-1];yaw=theta[-1];s0+=length
    return np.vstack(rows),definitions,lines


def build(output):
    anchor_path=WS/'config/Fitzpatrick_fixed_start.json'
    map_path=WS/'maps/Fitzpatrick.yaml'
    if output.exists() or Path(str(output)+'.report.json').exists():
        raise ValueError('Output exists; use a new version')
    anchor=json.loads(anchor_path.read_text())
    for name,digest in anchor['map_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Fixed-start map has changed')
    start=np.array([anchor['x'],anchor['y']]);heading=anchor['yaw_rad']
    field,ox,oy,res=_map_distance_field(map_path)
    # The map has a protrusion beside the lower part of the left straight.
    # Allow a tighter return corner so that straight stays clear of it.
    preferred=np.array([7.8,2.8,2.8,1.5,4.8])
    bounds=[(7.2,8.5),(2.0,3.4),(2.0,3.4),(1.3,2.5),(3.5,6.2)]
    def constraints(values):
        # Fixed count avoids changing SLSQP's constraint vector dimension.
        curve,_,lines=evaluate(values,start,heading,samples_per_segment=100)
        clearance=_field_clearance(curve[:,1:3],field,ox,oy,res)
        return np.r_[clearance-.80,np.array(lines)-np.array([2.,1.,6.])]
    def objective(values):
        scale=np.array([1.,1.,1.,.6,1.])
        # Prefer large-radius constant arcs while preserving the broad route
        # envelope. Map clearance constraints decide the final parameters.
        return float(np.sum(((values-preferred)/scale)**2)-.20*values[3])
    optimized=minimize(objective,preferred,method='SLSQP',bounds=bounds,
        constraints={'type':'ineq','fun':constraints},
        options={'maxiter':300,'ftol':1e-10,'disp':False})
    if not optimized.success or constraints(optimized.x).min()<-1e-6:
        raise ValueError('No feasible arc/straight layout: '+optimized.message)
    dense,segments,lines=evaluate(optimized.x,start,heading,step=.003)
    route,_,_=evaluate(optimized.x,start,heading,step=.05)
    total=float(dense[-1,0]);pose_error=float(np.linalg.norm(dense[-1,1:3]-start))
    yaw_error=abs(math.atan2(math.sin(dense[-1,3]-heading),math.cos(dense[-1,3]-heading)))
    k_error=float(abs(dense[-1,4]-dense[0,4]))
    hazards=cKDTree(map_hazards(map_path));distances=hazards.query(dense[:,1:3])[0]
    max_dense_step=float(np.max(np.linalg.norm(np.diff(dense[:,1:3],axis=0),axis=1)))
    clearance=float(distances.min()-res/math.sqrt(2)-max_dense_step/2)
    # Also constrain distance to the outside of the finite map.
    border=np.min(np.c_[dense[:,1]-ox,ox+field.shape[1]*res-dense[:,1],
                           dense[:,2]-oy,oy+field.shape[0]*res-dense[:,2]])
    clearance=min(clearance,float(border-max_dense_step/2))
    max_k=float(np.max(abs(dense[:,4])))
    steering=math.atan(.25*max_k)
    slopes=[abs(s['curvature_end_1pm']-s['curvature_start_1pm'])/s['length_m'] for s in segments]
    steering_rate_bound=.25*max(slopes)*2.0
    # Curvature >=0 and total turning exactly 2*pi describe a convex loop.
    # Independently check exported line segments for nonadjacent crossings.
    xy=route[:,1:3];a=xy[:-1];v=np.diff(xy,axis=0);crossings=[]
    for i in range(len(v)-2):
        j=np.arange(i+2,len(v))
        if i==0:j=j[j!=len(v)-1]
        q=a[j]-a[i];den=v[i,0]*v[j,1]-v[i,1]*v[j,0];valid=np.abs(den)>1e-10
        t=np.zeros(len(j));u=t.copy()
        t[valid]=(q[valid,0]*v[j[valid],1]-q[valid,1]*v[j[valid],0])/den[valid]
        u[valid]=(q[valid,0]*v[i,1]-q[valid,1]*v[i,0])/den[valid]
        crossings.extend((i,int(jj)) for jj in j[valid&(t>1e-7)&(t<1-1e-7)&(u>1e-7)&(u<1-1e-7)])
    if pose_error>1e-9 or yaw_error>1e-9 or k_error>1e-9 or clearance<.70 or steering>.269 or steering_rate_bound>1.0 or crossings:
        raise ValueError(f'Dense validation failed: closure={pose_error}, clearance={clearance}, steering={steering}, crossings={crossings}')
    # Remove quadrature round-off only, after checking true geometric closure.
    xy[-1]=xy[0]
    yaw=np.arctan2(np.sin(route[:,3]),np.cos(route[:,3]));yaw[-1]=yaw[0]
    poses=np.column_stack([xy,yaw])
    lengths={kind:sum(s['length_m'] for s in segments if s['kind']==kind) for kind in ('straight','arc','transition')}
    previous=WS/'routes/Fitzpatrick_fixed_start_closed_v1.csv'
    old=np.genfromtxt(previous,delimiter=',',skip_header=1)
    report=dict(geometry_ready=True,controller_ready=False,physical_tested=False,
        route_mode='closed_geometry_one_lap_stop_intended',length_m=total,poses=len(poses),
        fixed_start=[float(start[0]),float(start[1]),heading],end_pose_same_as_start=True,
        analytic_closure_position_error_m=pose_error,closure_heading_error_rad=yaw_error,
        closure_curvature_error_1pm=k_error,initial_arc_radius_m=float(optimized.x[0]),
        initial_constant_curvature_length_m=segments[0]['length_m'],
        straight_lengths_m=list(map(float,lines)),segment_lengths_by_type_m=lengths,
        constant_curvature_or_straight_fraction=(lengths['straight']+lengths['arc'])/total,
        transition_length_each_m=TRANSITION,maximum_curvature_1pm=max_k,
        required_max_steering_rad=steering,steering_rate_bound_at_2mps_radps=steering_rate_bound,
        minimum_hazard_cell_center_distance_m=float(distances.min()),
        conservative_static_clearance_m=clearance,dense_max_step_m=max_dense_step,self_intersections=crossings,
        maximum_distance_to_previous_route_m=float(cKDTree(old[:,:2]).query(dense[:,1:3])[0].max()),
        parameters=optimized.x.tolist(),segments=segments,
        integration_note='Do not substitute this closed route into the open-route-only candidate; single-lap progress, stopping and the time window still need integration.',
        map=str(map_path),anchor=str(anchor_path),output=str(output),
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [map_path,anchor_path,previous,Path(__file__).resolve()]})
    np.savetxt(output,poses,delimiter=',',header='x,y,yaw',comments='')
    Path(str(output)+'.report.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savez(str(output)+'.geometry.npz',s=route[:,0],xy=xy,yaw_unwrapped=route[:,3],curvature=route[:,4],segment=route[:,5],dense=dense)
    print(json.dumps({k:v for k,v in report.items() if k not in ('segments','source_sha256')},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=WS/'routes/Fitzpatrick_fixed_start_arc_straight_v2.csv')
    args=parser.parse_args();build(args.output)
