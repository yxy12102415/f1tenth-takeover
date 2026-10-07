"""Offline real-map ray-casting benchmark. No ROS node or device drivers.

Benchmarks variant-2 ray casting only, NOT full localization / DDS timing.
Output is synchronized by RangeLibc's device-to-host copy and synchronize.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from PIL import Image
import yaml
from nav_msgs.msg import OccupancyGrid

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'gpu_native_v1'))
import range_libc

def main():
    assert range_libc.SHOULD_USE_CUDA and Path(range_libc.__file__).parent==ROOT/'gpu_native_v1'
    cfg=yaml.safe_load((ROOT.parent/'maps/ps4_slam_20260904_163930.yaml').read_text())
    pixels=np.flipud(np.array(Image.open(cfg['image']),dtype=np.uint8))
    occupancy=1.-pixels.astype(float)/255.
    grid=np.full(pixels.shape,-1,dtype=np.int8)
    grid[occupancy<cfg['free_thresh']]=0;grid[occupancy>cfg['occupied_thresh']]=100
    msg=OccupancyGrid();msg.info.height,msg.info.width=grid.shape
    msg.info.resolution=cfg['resolution'];msg.info.origin.position.x=cfg['origin'][0]
    msg.info.origin.position.y=cfg['origin'][1];msg.info.origin.orientation.w=1.
    msg.data=grid.flatten().tolist()
    omap=range_libc.PyOMap(msg)
    assert not omap.error()
    route=np.loadtxt(ROOT.parent/'routes/manual_route_20260904_215501_controller_ready.csv',delimiter=',',skiprows=1)
    rng=np.random.RandomState(20260909)
    queries=route[np.arange(1000)%len(route)].astype(np.float32)
    queries[:,:2]+=rng.normal(0,.03,(1000,2)).astype(np.float32)
    # PF ray origins are laser poses, not base_link poses.
    queries[:,0]+=.27*np.cos(queries[:,2]);queries[:,1]+=.27*np.sin(queries[:,2])
    angles=np.linspace(-2.35,2.35,90,dtype=np.float32)
    assert len(queries)*len(angles)<=262144 # exclude upstream multi-chunk bug
    results={};outputs={}
    for name in ('cddt','rm','rmgpu'):
        method=(range_libc.PyCDDTCast(omap,200,112) if name=='cddt' else
                range_libc.PyRayMarching(omap,200) if name=='rm' else range_libc.PyRayMarchingGPU(omap,200))
        out=np.full(len(queries)*len(angles),np.nan,dtype=np.float32)
        for _ in range(5):method.calc_range_repeat_angles(queries,angles,out)
        times=[]
        for _ in range(100):
            start=time.perf_counter();method.calc_range_repeat_angles(queries,angles,out)
            times.append((time.perf_counter()-start)*1000.)
        assert np.isfinite(out).all() and (out>=0.).all() and (out<=10.001).all()
        outputs[name]=out.copy()
        results[name]={'median_ms':float(np.median(times)),'p95_ms':float(np.percentile(times,95)),
                       'max_ms':max(times),'finite_ranges':int(np.isfinite(out).sum())}
    for name in ('rm','cddt'):
        error=np.abs(outputs[name]-outputs['rmgpu'])
        results[name+'_vs_gpu']={'max_abs_m':float(error.max()),'p99_abs_m':float(np.percentile(error,99)),
                                'mean_abs_m':float(error.mean()),'over_5cm':int((error>.05).sum())}
    result={'cuda_compiled':True,'extension':range_libc.__file__,'particles':1000,'rays_per_particle':90,
            'iterations':100,'hardware_started':False,'full_pf_test':False,'results':results}
    print(json.dumps(result,indent=2))
    (ROOT/'gpu_benchmark.json').write_text(json.dumps(result,indent=2)+'\n')
    # Distinct approximate ray algorithms need an explicit localization review,
    # not a claim of interchangeable results based on finite outputs alone.
if __name__=='__main__':main()
