"""Create a bounded map-clearance revision of the recorded open route."""
from pathlib import Path
import math
import yaml
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, gaussian_filter, gaussian_filter1d, map_coordinates

WS=Path(__file__).resolve().parents[2]
SOURCE=WS/'routes/manual_route_20260904_215501_controller_ready.csv'
OUTPUT=WS/'routes/manual_route_20260904_215501_clearance_v1.csv'
MAP_YAML=WS/'maps/ps4_slam_20260904_163930.yaml'


def sample(field,points,origin,resolution):
    return map_coordinates(field,[(points[:,1]-origin[1])/resolution,
                                  (points[:,0]-origin[0])/resolution],
                           order=1,mode='constant',cval=0.)


def main():
    meta=yaml.safe_load(MAP_YAML.read_text())
    image_path=Path(meta['image'])
    if not image_path.is_absolute():image_path=MAP_YAML.parent/image_path
    pixels=np.asarray(Image.open(image_path).convert('L'))[::-1]
    occupancy=pixels/255. if meta.get('negate',0) else 1.-pixels/255.
    resolution=float(meta['resolution']);origin=meta['origin']
    distance=gaussian_filter(distance_transform_edt(~(occupancy>=meta['free_thresh']))*resolution,1.)
    grad_y,grad_x=np.gradient(distance,resolution,resolution)
    route=np.loadtxt(SOURCE,delimiter=',',skiprows=1)
    if route.shape!=(150,3) or not np.isfinite(route).all():raise RuntimeError('unexpected source route')
    original=route[:,:2].copy();points=original.copy()
    indices=np.arange(len(points))
    endpoint_taper=np.minimum(1.,np.minimum(indices/15.,(len(points)-1-indices)/15.))[:,None]
    for _ in range(150):
        clearance=sample(distance,points,origin,resolution)
        gradient=np.c_[sample(grad_x,points,origin,resolution),sample(grad_y,points,origin,resolution)]
        gradient/=np.maximum(np.linalg.norm(gradient,axis=1)[:,None],1.e-6)
        raw=points-original+.12*np.maximum(.54-clearance,0.)[:,None]*gradient
        displacement=np.c_[gaussian_filter1d(raw[:,0],7.,mode='nearest'),
                           gaussian_filter1d(raw[:,1],7.,mode='nearest')]
        norm=np.linalg.norm(displacement,axis=1)
        displacement*=np.minimum(1.,.10/np.maximum(norm,1.e-9))[:,None]
        points=original+endpoint_taper*displacement
    spacing=np.linalg.norm(np.diff(points,axis=0),axis=1)
    if spacing.min()<=1.e-4:raise RuntimeError('optimized route contains duplicate points')
    s=np.r_[0.,np.cumsum(spacing)]
    yaw=np.unwrap(route[:,2])
    curvature=np.abs(np.gradient(yaw,s))
    authority=math.tan(.27)/.25+.03
    if curvature.max()>authority:raise RuntimeError('optimized route exceeds steering authority')
    old_clearance=sample(distance,original,origin,resolution)
    new_clearance=sample(distance,points,origin,resolution)
    if np.percentile(new_clearance,10)<=np.percentile(old_clearance,10)+.04:raise RuntimeError('insufficient clearance gain')
    with OUTPUT.open('x') as out:
        out.write('x,y,yaw\n')
        for (x,y),heading in zip(points,route[:,2]):out.write(f'{x:.15f},{y:.15f},{heading:.15f}\n')
    print({'output':str(OUTPUT),'max_shift_m':float(np.linalg.norm(points-original,axis=1).max()),
           'clearance_p10_before_m':float(np.percentile(old_clearance,10)),
           'clearance_p10_after_m':float(np.percentile(new_clearance,10)),
           'median_before_m':float(np.median(old_clearance)),'median_after_m':float(np.median(new_clearance)),
           'max_curvature_per_m':float(curvature.max())})


if __name__=='__main__':main()
