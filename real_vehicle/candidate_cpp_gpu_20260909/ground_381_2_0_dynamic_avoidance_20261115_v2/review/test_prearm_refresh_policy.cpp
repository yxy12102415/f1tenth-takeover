#include "prearm_path_refresh.hpp"
#include <cassert>
#include <iostream>
using namespace frenet;
int main(){PrearmPathRefresh p;assert(p.update(false,0,{0,0,0}));
 for(int n=0;n<10000;++n)assert(!p.update(false,0,{.002*std::sin(n),.002*std::cos(n),.01*std::sin(n)}));
 assert(p.update(false,0,{.04,0,0}));assert(!p.update(false,0,{.041,0,0}));
 assert(!p.update(true,1.,{3.,0,1.}));assert(!p.update(false,.1,{3.,0,1.}));
 assert(p.update(false,0,{3.,0,1.}));assert(p.update(false,0,{3.,0,1.06}));
 PrearmPathRefresh wrapped;assert(wrapped.update(false,0,{0,0,3.131592653589793}));assert(!wrapped.update(false,0,{0,0,-3.131592653589793}));
 std::cout<<"{\"passed\":true,\"stationary_jitter_checks\":10000,\"timer_refresh_removed\":true,\"pose_change_refresh_kept\":true,\"active_path_not_reset\":true,\"physical_motion\":false}\n";
}
