#include "health_startup.hpp"
#include <cassert>
#include <iostream>
Json::Value sample(double t){Json::Value j;j["stamp"]=100.+t;j["erpm"]=0.;j["motor_current"]=0.;j["input_current"]=0.;j["fet_temp"]=25.;j["voltage"]=12.;j["duty_cycle"]=0.;j["fault"]=0.;return j;}
int main(){
  health::Startup s;
  assert(!s.update(sample(0),1.,100.));assert(!s.healthy(1.,100.));
  assert(!s.update(sample(.05),1.4,100.4));assert(!s.activated);
  for(int i=0;i<50;++i)assert(!s.update(sample(.5+i*.02),1.5+i*.02,100.5+i*.02));
  assert(s.update(sample(1.5),2.5,101.5));assert(s.activated);assert(s.healthy(2.51,101.51));
  assert(!s.healthy(3.001,102.001));assert(!s.update(sample(1.71),3.01,101.71));assert(s.reason()=="health_receipt_timeout");
  health::Startup bad;auto j=sample(0);j["fault"]=1.;assert(!bad.update(j,1.,100.));assert(bad.fatal);
  assert(!bad.update(sample(.02),1.02,100.02));assert(bad.reason()=="vesc_fault");
  health::Startup transient;j=sample(0);j["motor_current"]=40.;assert(!transient.update(j,1.,100.));
  for(int i=1;i<=50;++i){double t=i*.02;j=sample(t);j["motor_current"]=40.;
    bool ok=transient.update(j,1.+t,100.+t);assert(ok==(i==50));}
  assert(transient.activated);assert(transient.reason()=="healthy");
  health::Startup hot;j=sample(0);j["fet_temp"]=65.;assert(!hot.update(j,1.,100.));assert(hot.fatal);
  health::Startup never;
  for(int i=0;i<30;++i){double t=i*.2;assert(!never.update(sample(t),1.+t,100.+t));assert(!never.healthy(1.+t,100.+t));}
  assert(!never.activated);
  std::cout<<"Startup qualification, VESC fault and permanent runtime latch tests passed\n";
}
