#include <cassert>
#include <iostream>
#include <thread>
#include "takeover_sim/sqp_worker.hpp"
int main(int argc,char **argv) {
  assert(argc==2);
  takeover::SqpWorker worker(argv[1],"unused_test_fixture");
  Json::Value r;r["token"]="FIRST";
  auto first=worker.solve(r);
  assert(!first["ok"].asBool());assert(worker.timeouts==1);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  r["token"]="FRESH";
  auto second=worker.solve(r);
  assert(second["ok"].asBool());assert(second["token"].asString()=="FRESH");
  assert(worker.timeouts==1);assert(worker.failures==1);
  std::cout<<"Late/fragmented SQP result discarded; fresh solve recovered\n";
}
