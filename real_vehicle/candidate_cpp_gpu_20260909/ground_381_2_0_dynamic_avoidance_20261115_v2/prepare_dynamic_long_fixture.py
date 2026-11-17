"""Synthetic 15 m straight software fixture; never replaces the 381 map."""
from pathlib import Path
import json,numpy as np
from controller_assets import generate_profiles
R=Path(__file__).resolve().parent
D=R/'reports/dynamic_preparation/long_straight_fixture';D.mkdir(parents=True,exist_ok=True)
res=.05;width=400;height=200;ox=-2.;oy=-5.
x=ox+(np.arange(width)+.5)*res;y=oy+(height-1-np.arange(height)+.5)*res
clearance=np.minimum(np.minimum(x-ox,ox+width*res-x)[None,:],np.minimum(y-oy,oy+height*res-y)[:,None])
s=np.linspace(0,15,301);route=np.c_[s,np.zeros(len(s)),np.zeros(len(s))];speed=np.full(len(s),2.);speed[0]=speed[-1]=0.
a=dict(route=route.tolist(),speed=speed.tolist(),resolution=res,origin=[ox,oy],width=width,height=height,clearance=clearance.ravel().tolist(),endpoint_position_tolerance_m=.15,sources={},fixture_scope='Synthetic long straight; not the physical 381 map')
(D/'runtime_assets.json').write_text(json.dumps(a,separators=(',',':'))+'\n')
c=json.loads((R/'compact_config.json').read_text());c['physical_commissioning_enabled']=False;c['generic_max_lateral_offset_m']=.90;(D/'compact_config.json').write_text(json.dumps(c,indent=2)+'\n')
link=D/'install_cpp'
if not link.exists():link.symlink_to(R/'install_cpp',target_is_directory=True)
generate_profiles(D)
print(D)
