"""Offline-only map distance transform/profile export; no ROS or actuation."""
import json,hashlib
from pathlib import Path
import numpy as np
from return_lattice_planner import StaticMap,load_route
ROOT=Path(__file__).resolve().parent
WS=ROOT.parents[1]
route=WS/'routes/381_2_0_compact_overtake_595m_20261108_v4.csv'
profile=WS/'routes/381_2_0_compact_overtake_595m_20261108_v4_speed_profile.csv'
mapfile=WS/'maps/381_2_0.yaml'
r,closed=load_route(route)
assert not closed
m=StaticMap(mapfile)
v=np.genfromtxt(profile,delimiter=',',names=True)
print('profile columns',v.dtype.names)
# Profile carries x,y,yaw,s_m,curvature_1pm,speed_mps.
speed=v['speed_mps']
np.testing.assert_allclose(np.c_[v['x'],v['y'],v['yaw']],r,atol=1e-8,rtol=0)
assert len(speed)==len(r) and speed[0]==speed[-1]==0 and np.isfinite(speed).all() and speed.max()<=2.0
sources=[route,profile,mapfile,WS/'maps/381_2_0.pgm']
j=dict(route=r.tolist(),speed=speed.tolist(),resolution=m.resolution,origin=m.origin.tolist(),width=m.width,height=m.height,clearance=m.distance.reshape(-1).tolist(),sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
(ROOT/'runtime_assets.json').write_text(json.dumps(j,separators=(',',':'))+'\n')
