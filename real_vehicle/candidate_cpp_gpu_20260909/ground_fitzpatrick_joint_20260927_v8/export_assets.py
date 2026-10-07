"""Offline-only map distance transform/profile export; no ROS or actuation."""
import json,hashlib
from pathlib import Path
import numpy as np
from return_lattice_planner import StaticMap,load_route
ROOT=Path(__file__).resolve().parent
WS=ROOT.parents[1]
route=WS/'routes/Fitzpatrick_fixed_start_arc_straight_v2.csv'
profile=WS/'routes/Fitzpatrick_fixed_start_arc_straight_v2_speed_profile.csv'
mapfile=WS/'maps/Fitzpatrick.yaml'
r,closed=load_route(route)
assert closed
anchor_file=WS/"config/Fitzpatrick_fixed_start.json"
anchor=json.loads(anchor_file.read_text())
np.testing.assert_allclose(r[0],[anchor["x"],anchor["y"],anchor["yaw_rad"]],atol=1e-9,rtol=0)
np.testing.assert_allclose(r[-1],r[0],atol=1e-9,rtol=0)
m=StaticMap(mapfile)
v=np.genfromtxt(profile,delimiter=',',names=True)
print('profile columns',v.dtype.names)
# Profile carries x,y,yaw,s_m,curvature_1pm,speed_mps.
speed=v['speed_mps']
np.testing.assert_allclose(np.c_[v['x'],v['y'],v['yaw']],r,atol=1e-8,rtol=0)
assert len(speed)==len(r) and speed[0]==speed[-1]==0 and np.isfinite(speed).all() and speed.max()<=2.0
sources=[anchor_file,route,profile,mapfile,WS/'maps/Fitzpatrick.pgm']
j=dict(route_mode="closed_single_lap_stop",route=r.tolist(),speed=speed.tolist(),resolution=m.resolution,origin=m.origin.tolist(),width=m.width,height=m.height,clearance=m.distance.reshape(-1).tolist(),sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
(ROOT/'runtime_assets.json').write_text(json.dumps(j,separators=(',',':'))+'\n')
