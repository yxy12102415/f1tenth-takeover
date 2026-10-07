#!/usr/bin/env python3
"""Offline fixed-start circle + C2 spline loop; never enables a controller."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import minimize, minimize_scalar
from scipy.spatial import cKDTree

WS = Path(__file__).resolve().parent
sys.path.insert(0, str(WS/'src/f1tenth_safety_migration/f1tenth_safety_migration'))
from route_preprocessor import (_map_distance_field, _field_clearance,
                                map_hazards, read_route)


def basis(n, u, derivative=0):
    knots = np.r_[np.zeros(4), np.linspace(0, 1, n-2)[1:-1], np.ones(4)]
    return BSpline(knots, np.eye(n), 3).derivative(derivative)(u)


def build(raw_path, map_path, anchor_path, output, head_length):
    report_path = Path(str(output)+'.report.json')
    if output.exists() or report_path.exists():
        raise ValueError('Output already exists; choose a new version')
    raw = read_route(raw_path)
    anchor = json.loads(anchor_path.read_text())
    for path, checksum in anchor['map_sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != checksum:
            raise ValueError('Fixed-start map fingerprint changed')
    if str(map_path.resolve()) not in anchor['map_sha256']:
        raise ValueError('Map does not match the fixed start')
    start = np.array([anchor['x'], anchor['y']])
    heading = anchor['yaw_rad']
    if np.linalg.norm(raw[0,:2]-start) > .10:
        raise ValueError('Recording does not begin at the fixed start')
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(raw[:,:2],axis=0),axis=1))]
    if np.any(np.diff(arc) <= 1e-6) or arc[-1] < head_length+5:
        raise ValueError('Need an advancing route and enough length after the initial arc')
    direction = 1 if np.unwrap(raw[:,2])[np.searchsorted(arc,head_length)] > heading else -1
    def circle(s, radius):
        k = direction/radius
        angle = heading+k*np.asarray(s)
        return start+np.column_stack(((np.sin(angle)-math.sin(heading))/k,
                                     (math.cos(heading)-np.cos(angle))/k))
    fit_s = np.linspace(0, head_length, 100)
    fit_xy = np.column_stack([np.interp(fit_s,arc,raw[:,i]) for i in (0,1)])
    fit = minimize_scalar(lambda r: np.mean((circle(fit_s,r)-fit_xy)**2),
                          bounds=(2.,15.), method='bounded')
    radius = float(fit.x)
    k = direction/radius
    join = circle([head_length],radius)[0]
    join_heading = heading+k*head_length
    gap = float(np.linalg.norm(raw[-1,:2]-start))
    span = arc[-1]-head_length+gap
    n = max(18, int(math.ceil(span/.85))+3)
    anchors_u = np.array([0.,1.])
    # Fix position, tangent and second derivative at both ends. The circular
    # part uses arc length; parameter derivatives are scaled by `span` here.
    equality = np.vstack([basis(n,anchors_u,d) for d in (0,1,2)])
    tangent = np.array([[math.cos(join_heading),math.sin(join_heading)],
                        [math.cos(heading),math.sin(heading)]])
    normal = np.column_stack([-tangent[:,1],tangent[:,0]])
    values = np.vstack([join,start,span*tangent,k*span*span*normal])
    fixed = np.linalg.lstsq(equality, values, rcond=None)[0]
    freedom = null_space(equality)
    u = np.linspace(0,1,max(250,int(span/.08)))
    source_s = head_length+u*span
    source_arc = np.r_[arc,arc[-1]+gap]
    source_xy = np.vstack([raw[:,:2],start])
    target = np.column_stack([np.interp(source_s,source_arc,source_xy[:,i]) for i in (0,1)])
    b = basis(n,u)
    regularizer = np.diff(np.eye(n),n=2,axis=0)
    system = np.vstack([b@freedom, .3*regularizer@freedom])
    rhs = np.vstack([target-b@fixed,-.3*regularizer@fixed])
    initial = np.linalg.lstsq(system,rhs,rcond=None)[0]
    field,ox,oy,res = _map_distance_field(map_path)
    sample_u = np.linspace(0,1,max(600,int(span/.05)))
    b0,b1,b2 = [basis(n,sample_u,d) for d in (0,1,2)]
    limit = .95*math.tan(.269)/.25
    def controls(z): return fixed+freedom@z.reshape(-1,2)
    def constraints(z):
        c=controls(z);xy=b0@c;v=b1@c;a=b2@c
        speed=np.linalg.norm(v,axis=1)
        curvature=(v[:,0]*a[:,1]-v[:,1]*a[:,0])/np.maximum(speed**3,1e-12)
        clearance=_field_clearance(xy,field,ox,oy,res)
        return np.r_[limit-np.abs(curvature),clearance-.75,speed-.25*span]
    def objective(z):
        c=controls(z);error=b@c-target;bend=regularizer@c
        return np.mean(error**2)+.02*np.mean(bend**2)
    optimized=minimize(objective,initial.ravel(),method='SLSQP',
        constraints={'type':'ineq','fun':constraints},
        options={'maxiter':400,'ftol':1e-10,'disp':False})
    if not optimized.success or constraints(optimized.x).min() < -1e-5:
        raise ValueError('No feasible map/curvature solution: '+optimized.message)
    c=controls(optimized.x)
    dense_u=np.linspace(0,1,max(6001,int(span/.004)))
    xy=basis(n,dense_u)@c;d1=basis(n,dense_u,1)@c;d2=basis(n,dense_u,2)@c
    spline_yaw=np.unwrap(np.arctan2(d1[:,1],d1[:,0]))
    spline_yaw += 2*math.pi*round((join_heading-spline_yaw[0])/(2*math.pi))
    spline_k=(d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])/np.linalg.norm(d1,axis=1)**3
    head_s=np.linspace(0,head_length,int(math.ceil(head_length/.004))+1)
    dense=np.vstack([circle(head_s,radius),xy[1:]])
    dense_yaw=np.r_[heading+k*head_s,spline_yaw[1:]]
    dense_k=np.r_[np.full(len(head_s),k),spline_k[1:]]
    dense_s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(dense,axis=0),axis=1))]
    length=float(dense_s[-1]);samples=np.linspace(0,length,int(math.ceil(length/.05))+1)
    route=np.column_stack([np.interp(samples,dense_s,dense[:,i]) for i in (0,1)])
    yaw=np.interp(samples,dense_s,dense_yaw)
    route[0]=route[-1]=start
    yaw[0]=heading;yaw[-1]=heading+2*math.pi*direction
    poses=np.column_stack([route,np.arctan2(np.sin(yaw),np.cos(yaw))])
    tree=cKDTree(map_hazards(map_path));distances=tree.query(dense)[0]
    max_step=float(np.linalg.norm(np.diff(dense,axis=0),axis=1).max())
    # Conservative distance to occupied/unknown cell squares, accounting for
    # half a dense-sample interval. Report this separately from cell centers.
    conservative=float(distances.min()-res/math.sqrt(2)-max_step/2)
    all_k=np.r_[np.diff(yaw)/np.diff(samples),dense_k]
    steering=float(math.atan(.25*np.max(np.abs(all_k))))
    # Verify polyline simplicity, excluding the adjacent closing segments.
    a=route[:-1];v=np.diff(route,axis=0);intersections=[]
    for i in range(len(v)-2):
        j=np.arange(i+2,len(v))
        if i==0:j=j[j!=len(v)-1]
        diff=a[j]-a[i];den=v[i,0]*v[j,1]-v[i,1]*v[j,0]
        valid=np.abs(den)>1e-10
        t=np.zeros(len(j));q=t.copy()
        t[valid]=(diff[valid,0]*v[j[valid],1]-diff[valid,1]*v[j[valid],0])/den[valid]
        q[valid]=(diff[valid,0]*v[i,1]-diff[valid,1]*v[i,0])/den[valid]
        hit=valid&(t>1e-7)&(t<1-1e-7)&(q>1e-7)&(q<1-1e-7)
        intersections += [(i,int(x)) for x in j[hit]]
    if conservative<.70 or steering>.269 or intersections:
        raise ValueError(f'Dense validation failed: clearance={conservative}, steering={steering}, intersections={intersections}')
    report=dict(route_mode='closed_geometry_one_lap_stop_intended',geometry_ready=True,
        controller_ready=False,controller_integration_note='Current Fitzpatrick candidate accepts open routes only; closed progress and lap completion must be integrated before driving.',
        length_m=length,poses=len(poses),spacing_max_m=float(np.max(np.linalg.norm(np.diff(route,axis=0),axis=1))),
        fixed_start=[float(start[0]),float(start[1]),heading],closure_position_error_m=float(np.linalg.norm(route[-1]-route[0])),
        closure_heading_error_rad=float(abs(math.atan2(math.sin(yaw[-1]-yaw[0]),math.cos(yaw[-1]-yaw[0])))),
        first_arc_length_m=head_length,first_arc_radius_m=radius,first_arc_curvature_1pm=k,
        seam_curvature_error_1pm=float(abs(spline_k[-1]-k)),arc_join_curvature_error_1pm=float(abs(spline_k[0]-k)),
        maximum_curvature_1pm=float(np.max(np.abs(all_k))),required_max_steering_rad=steering,
        minimum_hazard_cell_center_distance_m=float(distances.min()),
        conservative_static_clearance_m=conservative,dense_max_step_m=max_step,self_intersections=intersections,
        raw_length_m=float(arc[-1]),raw_closure_gap_m=gap,
        maximum_raw_to_smoothed_distance_m=float(cKDTree(dense).query(raw[:,:2])[0].max()),
        maximum_smoothed_to_raw_distance_m=float(cKDTree(raw[:,:2]).query(dense)[0].max()),
        optimizer_iterations=int(optimized.nit),input=str(raw_path.resolve()),map=str(map_path.resolve()),
        fixed_start_file=str(anchor_path.resolve()),output=str(output.resolve()),
        source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in (raw_path,map_path,anchor_path)})
    output.parent.mkdir(parents=True,exist_ok=True)
    np.savetxt(output,poses,delimiter=',',header='x,y,yaw',comments='')
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    np.savez(str(output)+'.geometry.npz',s=samples,xy=route,yaw_unwrapped=yaw,
             curvature=np.interp(samples,dense_s,dense_k),dense_xy=dense)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=WS/'routes/manual_route_20260927_204931.csv')
    parser.add_argument('--map',type=Path,default=WS/'maps/Fitzpatrick.yaml')
    parser.add_argument('--anchor',type=Path,default=WS/'config/Fitzpatrick_fixed_start.json')
    parser.add_argument('--output',type=Path,default=WS/'routes/Fitzpatrick_fixed_start_closed_v1.csv')
    parser.add_argument('--head-length',type=float,default=6.)
    args=parser.parse_args()
    build(args.input,args.map,args.anchor,args.output,args.head_length)
