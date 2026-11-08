#include "firmware/include/link_core.h"
#include <cassert>
#include <iostream>
using namespace opp;
Frame cmd(uint32_t seq, uint32_t time, bool enable=false) {
  Frame f; f.type=Command; f.boot=123; f.session=456; f.sequence=seq;
  f.tick=time; f.flags=enable; f.motor=enable?1100:kStop; return f;
}
Core running() {
  Core c(123); assert(c.accept(cmd(1,6000),6000));
  assert(c.state==Ready); assert(c.accept(cmd(2,6020,true),6020));
  assert(c.state==Running && c.applied()==1000); return c;
}
int main() {
  { Core c(123); assert(!c.accept(cmd(1,6000,true),6000)); assert(c.session==0); }
  { Core c(123); assert(c.accept(cmd(1,1000),1000));
    assert(c.accept(cmd(2,1020,true),1020)); assert(c.state==WaitingRelease && c.applied()==1000); }
  { auto c=running(); assert(c.accept(cmd(3,6040),6040)); assert(c.state==Latched);
    assert(c.accept(cmd(4,6060,true),6060)); assert(c.state==Latched && c.requested==1000); }
  { auto c=running(); c.tick(6270); assert(c.state==Latched);
    assert(c.accept(cmd(3,6270,true),6270)); assert(c.state==Latched); }
  { auto c=running(); for(uint32_t t=6040;t<=6280;t+=20) c.accept(cmd(2,6020,true),t);
    assert(c.state==Latched); }
  { auto c=running(); auto f=cmd(3,6040,true); f.session=999;
    assert(!c.accept(f,6040)); f=cmd(3,6040,true); f.boot=999;
    assert(!c.accept(f,6040)); c.tick(6270); assert(c.state==Latched); }
  { auto c=running(); auto f=cmd(3,6040,true); f.motor=1200;
    assert(!c.accept(f,6040)); assert(c.state==Latched && c.applied()==1000); }
  { auto c=running(); auto f=cmd(3,6040,true); f.steering=1601;
    assert(!c.accept(f,6040)); assert(c.state==Latched); }
  { auto c=running(); auto f=cmd(3,5500,true); assert(!c.accept(f,6040));
    c.tick(6270); assert(c.state==Latched); }
  { auto c=running(); for(uint32_t i=3,t=6040;t<=11040;++i,t+=20) c.accept(cmd(i,t,true),t);
    assert(c.state==Latched); }
  { Core c(123); uint32_t start=0xffffff00;
    assert(c.accept(cmd(1,start),start)); assert(c.accept(cmd(2,start+20,true),start+20));
    c.tick(20); assert(c.state==Latched); }
  std::cout << "11 receiver state-machine scenarios passed; motor output always inhibited\n";
}
