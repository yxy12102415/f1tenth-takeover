"""Offline-only map distance transform/profile export; no ROS or actuation."""
import json,hashlib
from pathlib import Path
import numpy as np
from return_lattice_planner import StaticMap,load_route
ROOT=Path(__file__).resolve().parent
WS=ROOT.parents[1]
route=WS/'routes/manual_route_20260923_172359_controller_ready.csv'
profile=WS/'routes/manual_route_20260923_172359_controller_ready_speed_profile_v2.csv'
mapfile=WS/'maps/381_2_0.yaml'
r,closed=load_route(route)
assert not closed
m=StaticMap(mapfile)
v=np.genfromtxt(profile,delimiter=',',names=True)
print('profile columns',v.dtype.names)
# Profile carries x,y,yaw,s_m,curvature_1pm,speed_mps.
speed=np.minimum(2.,v['speed_mps']*(2/1.3363440021))
np.testing.assert_allclose(np.c_[v['x'],v['y'],v['yaw']],r,atol=1e-8,rtol=0)
assert len(speed)==len(r) and speed[0]==speed[-1]==0 and np.isfinite(speed).all() and speed.max()<=2.0
sources=[route,profile,mapfile,WS/'maps/381_2_0.pgm']
j=dict(route=r.tolist(),speed=speed.tolist(),resolution=m.resolution,origin=m.origin.tolist(),width=m.width,height=m.height,clearance=m.distance.reshape(-1).tolist(),sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
position_tolerance=json.loads((ROOT/'compact_config.json').read_text()).get('endpoint_position_tolerance_m',.10)
assert .10<=position_tolerance<=.15
j['endpoint_position_tolerance_m']=position_tolerance
wall_margin=json.loads((ROOT/'compact_config.json').read_text()).get('wall_margin_m',.05)
assert .02<=wall_margin<=.10
j['wall_margin_m']=wall_margin
j['speed_derivation']=dict(method='multiply_source_profile_then_cap',factor=2/1.3363440021,cap_mps=2.,running_command_floor_mps=.5,source_profile_unchanged=True)
(ROOT/'runtime_assets.json').write_text(json.dumps(j,separators=(',',':'))+'\n')
