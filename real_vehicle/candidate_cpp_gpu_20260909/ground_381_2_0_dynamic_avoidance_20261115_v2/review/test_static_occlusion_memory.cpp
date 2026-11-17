#include "tracked_occupancy.hpp"
#include <iostream>
using namespace frenet;
void require(bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);}
Track component(double stamp,double x){Track t;t.stamp=stamp;t.hits=5;t.cluster.points={Point(x,-.1),Point(x+.1,-.1),Point(x+.1,.1),Point(x,.1)};return t;}
int main(int argc,char**argv){try{
 Assets mapped(argv[1]);std::fill(mapped.grid.begin(),mapped.grid.end(),.1);Pose pose{0,0,0};std::vector<float> blocked(1081,.3);
 auto observe=[&](DynamicMemory& memory,double stamp,std::vector<Track> tr){memory.observe(tr,stamp,blocked,-2.35619449,4.71238898/1080,.05,10.,pose);};
 DynamicMemory stationary(mapped);for(int n=0;n<20;++n)observe(stationary,100+n*.05,{component(100+n*.05,1.5)});
 auto before=stationary.snapshot()[0];require(before["map_supported"].asBool(),"fixture_map_support");
 for(int n=20;n<55;++n)observe(stationary,100+n*.05,{});
 auto after=stationary.snapshot();require(after.size()==1,"static_occlusion_forgotten");auto a=after[0];
 require(a["fixed_world"].asBool() && a["memory_state"]=="occluded_static","static_occlusion_class_lost");
 for(auto key:{"xmin","xmax","ymin","ymax","stamp"})require(a[key]==before[key],"static_occlusion_added_padding_or_renewed_stamp");
 DynamicMemory moving(mapped);for(int n=0;n<40;++n)observe(moving,200+n*.05,{component(200+n*.05,1.+.5*n*.05)});
 auto m=moving.snapshot()[0];require(m["map_supported"].asBool() && m["has_moved"].asBool(),"mapped_mover_fixture");
 for(int n=40;n<75;++n)observe(moving,200+n*.05,{});
 auto lost=moving.snapshot();require(lost.size()==1 && !lost[0]["fixed_world"].asBool() && lost[0]["has_moved"].asBool() && lost[0]["memory_state"]=="occluded_unknown","mapped_mover_mistaken_for_wall");
 Assets unmap=mapped;std::fill(unmap.grid.begin(),unmap.grid.end(),2.);DynamicMemory unknown(unmap);
 for(int n=0;n<20;++n)observe(unknown,300+n*.05,{component(300+n*.05,1.5)});
 for(int n=20;n<55;++n)observe(unknown,300+n*.05,{});
 auto u=unknown.snapshot();require(u.size()==1 && u[0]["fixed_world"].asBool() && u[0]["memory_state"]=="occluded_static","confirmed_unmapped_static_not_preserved");
 DynamicMemory tentative(unmap);observe(tentative,400.,{component(400.,1.5)});observe(tentative,400.15,{});auto tt=tentative.snapshot();require(tt.size()==1 && !tt[0]["fixed_world"].asBool(),"unconfirmed_target_frozen");
 // Reobservation supersedes the fixed occlusion hypothesis, and coherent
 // displacement must switch back to moving prediction.
 for(int n=0;n<40;++n)observe(unknown,303.+n*.05,{component(303.+n*.05,1.5+.5*n*.05)});
 auto changed=unknown.snapshot();require(changed.size()==1 && changed[0]["has_moved"].asBool() && !changed[0]["fixed_world"].asBool(),"static_target_failed_to_resume_motion");
 Json::Value out;out["passed"]=true;out["physical_motion"]=false;out["map_static_no_occlusion_growth"]=true;out["source_stamp_preserved"]=true;out["mapped_moving_history_preserved"]=true;out["confirmed_unmapped_static_held"]=true;out["unconfirmed_not_frozen"]=true;out["static_reobservation_switches_to_motion"]=true;std::cout<<dump(out)<<std::endl;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
