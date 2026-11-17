#include "health_startup.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cstdlib>
#include <sstream>
static double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class HealthNode:public rclcpp::Node {
  health::Startup core;bool physical;unsigned samples=0;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permit;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  rclcpp::TimerBase::SharedPtr timer,status_timer;
public:
  explicit HealthNode(bool real):rclcpp::Node("ground_health_interlock",rclcpp::NodeOptions().start_parameter_services(false)),physical(real){
    permit=create_publisher<std_msgs::msg::Bool>("/stand_pf/health_permit",1);
    status=create_publisher<std_msgs::msg::String>("/stand_pf/health_guard_status",1);
    sub=create_subscription<std_msgs::msg::String>("/stand_pf/health",1,[this](std_msgs::msg::String::ConstSharedPtr m){
      try{
        Json::Value j;Json::CharReaderBuilder b;b["allowSpecialFloats"]=true;std::string error;std::istringstream stream(m->data);
        if(!Json::parseFromStream(b,stream,&j,&error)||!j.isObject()){core.trip("invalid_health_payload");return;}
        if(physical&&j.isMember("synthetic")&&j["synthetic"]!=Json::Value(false)){core.trip("synthetic_health_forbidden");return;}
        ++samples;core.update(j,mono(),now().seconds());
      }catch(...){core.trip("invalid_health_payload");}
    });
    timer=create_wall_timer(std::chrono::milliseconds(20),[this](){std_msgs::msg::Bool b;b.data=core.healthy(mono(),now().seconds());permit->publish(b);});
    status_timer=create_wall_timer(std::chrono::milliseconds(20),[this](){
      bool ok=core.healthy(mono(),now().seconds());Json::Value j;
      j["healthy"]=ok;j["reason"]=core.reason();
      j["motor_temperature_used"]=false;j["samples"]=samples;j["qualified"]=core.activated;j["startup_rejected_samples"]=core.rejected;
      j["receipt_age_s"]=core.runtime.has?mono()-core.runtime.received:1.e9;
      j["source_age_s"]=core.runtime.has?now().seconds()-core.runtime.stamp:1.e9;
      Json::StreamWriterBuilder w;w["indentation"]="";std_msgs::msg::String s;s.data=Json::writeString(w,j);status->publish(s);
    });
  }
};
int main(int argc,char**argv){
  const char*d=std::getenv("ROS_DOMAIN_ID"),*l=std::getenv("ROS_LOCALHOST_ONLY");
  if(!d||!l||std::string(l)!="1"||(std::string(d)!="83"&&std::string(d)!="85"))return 2;
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<HealthNode>(std::string(d)=="85"));}
  catch(const std::exception&e){std::fprintf(stderr,"%s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
