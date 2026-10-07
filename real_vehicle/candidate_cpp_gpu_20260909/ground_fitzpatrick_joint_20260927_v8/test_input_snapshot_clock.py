"""Compile real old/new freshness methods under a deterministic clock schedule.
Models a callback receipt occurring between loop time and snapshot acquisition.
Also verifies that genuine stale, future or invalid inputs remain rejected.
"""
from pathlib import Path
import subprocess,json,tempfile,hashlib
ROOT=Path(__file__).resolve().parent
new=(ROOT/'src/cpp_frenet/node.cpp').read_text();old=(ROOT.with_name('ground_fitzpatrick_joint_20260927_v6')/'src/cpp_frenet/node.cpp').read_text()
def method(source,signature):
 start=source.index(signature);pos=source.index('{',start);depth=1;end=pos+1
 while depth:
  depth+=(source[end]=='{')-(source[end]=='}');end+=1
 return source[start:end]
header='''#include <cmath>
#include <cassert>
#include <tuple>
#include <iostream>
#include <jsoncpp/json/json.h>
double clock_mono=10.001,clock_ros=100.;
double mono(){return clock_mono;}
struct FakeTime{double seconds(){return clock_ros;}};
struct FakeClock{FakeTime now(){return {};}};
struct Input { bool scan=true,quality=true; double ps=100.,ss=100.,os=100.,pr=10.0005,sr=10.0005,orr=10.0005,qr=10.0005; };
'''
def klass(name,text):
 return 'struct '+name+' { Json::Value input_failure_snapshot; FakeClock clock; FakeClock* get_clock(){return &clock;} '+method(text,'  bool fresh(')+method(text,'  bool ready(')+' };\n'
main='''int main(){Legacy a; Fixed b;Input i;
// Exact interleaving: begin=10; callback receipts=10.0005; snapshot+check=10.001.
assert(!a.ready(i,10.));assert(b.ready(i));
for(int k=0;k<7;++k){Input q=i;
 if(k==0)q.scan=false;
 if(k==1)q.quality=false;
 if(k==2)q.ps=99.49;
 if(k==3)q.sr=9.5;
 if(k==4)q.orr=10.002;
 if(k==5)q.qr=9.5;
 if(k==6)q.os=100.021;
 assert(!b.ready(q));assert(b.input_failure_snapshot.isObject());}
std::cout<<"clock_order_and_seven_failure_cases_passed\\n";
}'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.cpp').write_text(header+klass('Legacy',old)+klass('Fixed',new)+main);subprocess.run(['g++','-std=c++14','-O2',str(p/'test.cpp'),'-ljsoncpp','-o',str(p/'test')],check=True);subprocess.run([str(p/'test')],check=True)
# Audit every runtime call site: no stale loop time passed to ready.
assert 'ready(i, begin)' not in new and 'ready(snapshot(), mono())' not in new
r=dict(passed=True,legacy_false_stale_reproduced=True,fixed_snapshot_passed=True,genuine_fault_cases_rejected=7,timeout_unchanged_s=.5,new_source_sha256=hashlib.sha256(new.encode()).hexdigest(),scope='deterministic reproduction of code-level race, not proof of unrecorded physical planner inputs')
(ROOT/'run_logs/input_snapshot_clock.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
