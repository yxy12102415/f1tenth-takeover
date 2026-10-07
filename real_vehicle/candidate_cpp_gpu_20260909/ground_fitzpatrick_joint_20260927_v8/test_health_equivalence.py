"""Compare the C++ health state machine with the existing Python reference."""
import json,math,random,subprocess
from pathlib import Path
from health_guard_core import HealthGuard
from health_interlock import LIMITS
base=dict(stamp=100.,erpm=0.,motor_current=0.,input_current=0.,fet_temp=25.,voltage=12.,duty_cycle=0.,fault=0,motor_temp=-81.9)
rng=random.Random(10);commands=[];expected=[]
def run(sequence):
    commands.append(dict(op='reset'));core=HealthGuard(LIMITS)
    for cmd in sequence:
        commands.append(cmd)
        value=core.update(cmd['sample'],cmd['mono'],cmd['ros']) if cmd['op']=='update' else core.healthy(cmd['mono'],cmd['ros'])
        expected.append(dict(value=value,reason=core.reason))
def update(t,**fields):return dict(op='update',sample=dict(base,stamp=100.+t,**fields),mono=1.+t,ros=100.+t)
def tick(t):return dict(op='tick',mono=1.+t,ros=100.+t)
run([tick(0),update(0),tick(.149),tick(.151),update(.16),tick(.17)])
for field,values in {
    'motor_current':[29.9,30.,63.,63.001,-64.,float('nan'),float('inf'),True,None],
    'input_current':[40.,40.01,-41.], 'fet_temp':[0.,.1,59.999,60.,65.],
    'voltage':[10.49,10.5,12.], 'erpm':[499.,500.,4500.,4500.01,-4501.],
    'duty_cycle':[-1.,1.,1.01], 'fault':[0,1], 'motor_temp':[None,float('nan'),-81.9]
}.items():
    for value in values:run([update(0),update(.05,**{field:value}),tick(.06),update(.10),tick(.11)])
run([update(0,motor_current=40.),update(.05,motor_current=40.),update(.11,motor_current=40.),update(.12),tick(.13)])
run([update(0),update(.05),update(.05),tick(.06)])
run([update(0),update(.151),tick(.152)])
run([update(.05),update(0),tick(.06)])
for age in [-.021,-.019,.149,.151]:
    cmd=update(0);cmd['ros']+=age;run([cmd,tick(.01)])
for key in base:
    cmd=update(0);del cmd['sample'][key];run([cmd,tick(.01)])
for trial in range(100):
    seq=[]
    for i in range(40):
        t=i*.025
        fields=dict(erpm=rng.uniform(600,4000),motor_current=rng.uniform(-20,20),input_current=rng.uniform(-10,10))
        if i==20:fields[rng.choice(['motor_current','input_current','fet_temp','fault'])]=rng.choice([0.,10.,70.])
        seq.extend([update(t,**fields),tick(t+.01)])
    run(seq)
exe=Path(__file__).parent/'install_v1/ground_pp/lib/ground_pp/health_core_probe'
result=subprocess.run([str(exe)],input='\n'.join(json.dumps(c) for c in commands)+'\n',text=True,capture_output=True,check=True)
actual=[json.loads(line) for line in result.stdout.splitlines()]
assert len(actual)==len(expected),(len(actual),len(expected))
for i,(a,e) in enumerate(zip(actual,expected)):
    assert a==e,(i,a,e)
print(json.dumps(dict(passed=True,comparisons=len(actual),includes_nonfinite_ignored_motor_temperature=True)))
