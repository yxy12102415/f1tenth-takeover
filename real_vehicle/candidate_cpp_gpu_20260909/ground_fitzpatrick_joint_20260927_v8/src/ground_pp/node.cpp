#include "core.hpp"
#include <cstdlib>
#include <chrono>
#include <memory>
#include <Eigen/Eigenvalues>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
using Odom=nav_msgs::msg::Odometry;
using Scan=sensor_msgs::msg::LaserScan;
using Bool=std_msgs::msg::Bool;
using String=std_msgs::msg::String;
using Drive=ackermann_msgs::msg::AckermannDriveStamped;
static double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static double stamp(const builtin_interfaces::msg::Time &s){return s.sec+s.nanosec*1.e-9;}
static double yaw_checked(const geometry_msgs::msg::Quaternion &q){
  double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(norm)||std::abs(norm-1.)>.02||std::abs(q.x)>.01||std::abs(q.y)>.01)throw std::runtime_error("invalid quaternion");
  return 2.*std::atan2(q.z,q.w);
}
class Node:public rclcpp::Node {
  candidate::Core core;
  candidate::Freshness pose_clock,scan_clock,odom_clock;
  bool pose_ok=false,scan_ok=false,odom_ok=false,quality=false,enabled=false,last_route_valid=false;
  double quality_time=-1.,enable_time=-1.,measured=0.,clearance=0.,previous=-1.;
  unsigned sequence=0;
  rclcpp::Publisher<Drive>::SharedPtr drive;
  rclcpp::Publisher<Bool>::SharedPtr valid;
  rclcpp::Publisher<String>::SharedPtr status;
  rclcpp::Subscription<Odom>::SharedPtr pose_sub,odom_sub;
  rclcpp::Subscription<Scan>::SharedPtr scan_sub;
  rclcpp::Subscription<Bool>::SharedPtr quality_sub,enable_sub;
  rclcpp::TimerBase::SharedPtr watchdog;
  std::string unhealthy(double n,double r) {
    if(!pose_clock.fresh(r,n))return "pose_stale";
    if(!scan_clock.fresh(r,n))return "scan_stale";
    if(!odom_clock.fresh(r,n))return "odom_stale";
    if(!pose_ok||!quality||n-quality_time>.5)return "localization_invalid";
    if(!scan_ok)return "scan_invalid";
    if(!odom_ok)return "odom_invalid";
    if(clearance<.45)return "front_obstacle";
    return "";
  }
  void publish(candidate::Decision d,double n,const std::string &event) {
    Drive out;out.header.stamp=now();out.header.frame_id="base_link";
    out.drive.speed=candidate::encode(d.speed,1.75);out.drive.steering_angle=candidate::encode(d.angle,.27);
    drive->publish(out);Bool b;b.data=d.valid;valid->publish(b);
    std::ostringstream ss;ss.precision(17);
    ss<<"{\"valid\":"<<(d.valid?"true":"false")<<",\"complete\":"<<(d.complete?"true":"false")
      <<",\"reason\":\""<<d.reason<<"\",\"controller\":\"cpp_pose_callback_candidate\",\"speed_command\":"<<out.drive.speed
      <<",\"steering_command\":"<<out.drive.steering_angle<<",\"progress_index\":"<<core.index
      <<",\"event\":\""<<event<<"\",\"runtime_timing\":{\"sequence\":"<<++sequence<<",\"tick_start_mono\":"<<n
      <<",\"publish_mono\":"<<mono()<<"},\"input_age\":{";
    bool comma=false;
    for(auto entry:{std::make_pair("pose",pose_clock),std::make_pair("scan",scan_clock),std::make_pair("odom",odom_clock)}){
      if(comma)ss<<",";
      comma=true;
      ss<<"\""<<entry.first<<"\":{\"source_age_s\":"<<(entry.second.stamp>0.?now().seconds()-entry.second.stamp:1.e9)
        <<",\"receipt_age_s\":"<<(entry.second.receipt>0.?n-entry.second.receipt:1.e9)<<",\"timeout_s\":0.5}";
    }
    ss<<"}}";String s;s.data=ss.str();status->publish(s);
  }
  void stop(const std::string &why){last_route_valid=false;publish(core.invalid(why),mono(),"stop_watchdog");}
  void operator_stop(){
    auto reason=unhealthy(mono(),now().seconds());
    if(!reason.empty()){stop(reason);return;}
    candidate::Decision d;d.valid=last_route_valid;d.complete=core.complete;
    d.reason="waiting_for_operator";core.last_speed=0.;publish(d,mono(),"operator_stop");
  }
 public:
  Node():rclcpp::Node("stand_cpp_pp",rclcpp::NodeOptions().start_parameter_services(false)),
    core(candidate::load(declare_parameter<std::string>("route_file",""))) {
    drive=create_publisher<Drive>("/baseline/drive_nominal",1);
    valid=create_publisher<Bool>("/baseline/planner_valid",1);
    status=create_publisher<String>("/baseline/controller_status",1);
    pose_sub=create_subscription<Odom>("/baseline/localization_pose",1,[this](Odom::ConstSharedPtr m){
      double n=mono();
      try{
        if(m->header.frame_id!="map"||m->child_frame_id!="base_link")throw std::runtime_error("pose frame");
        double yaw=yaw_checked(m->pose.pose.orientation),x=m->pose.pose.position.x,y=m->pose.pose.position.y;
        if(!std::isfinite(x)||!std::isfinite(y))throw std::runtime_error("pose finite");
        Eigen::Matrix3d cov;int ix[3]={0,1,5};
        for(double v:m->pose.covariance)if(!std::isfinite(v))throw std::runtime_error("cov finite");
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)cov(i,j)=m->pose.covariance[ix[i]*6+ix[j]];
        if((cov-cov.transpose()).cwiseAbs().maxCoeff()>1.e-8)throw std::runtime_error("cov asymmetric");
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> xy(cov.topLeftCorner<2,2>());
        if(eig.info()!=Eigen::Success||xy.info()!=Eigen::Success||eig.eigenvalues().minCoeff()< -1.e-8||xy.eigenvalues().maxCoeff()>.0625||cov(2,2)>.09)throw std::runtime_error("cov uncertainty");
        if(!pose_clock.accept(stamp(m->header.stamp),now().seconds(),n))throw std::runtime_error("pose timestamp");
        pose_ok=true;auto reason=unhealthy(n,now().seconds());
        if(!reason.empty()){stop(reason);return;}
        double dt=previous<0.?.05:n-previous;previous=n;
        auto d=core.step(x,y,yaw,measured,dt);
        last_route_valid=d.valid;
        if(d.valid&&(!enabled||n-enable_time>.50)){d.speed=d.angle=0.;d.reason="waiting_for_operator";core.last_speed=0.;}
        publish(d,n,"pose_callback");
      }catch(const std::exception &){pose_ok=false;stop("pose_invalid");}
    });
    odom_sub=create_subscription<Odom>("/baseline/odom",1,[this](Odom::ConstSharedPtr m){
      try{
        yaw_checked(m->pose.pose.orientation);
        if(m->header.frame_id!="odom"||m->child_frame_id!="base_link"||!std::isfinite(m->pose.pose.position.x)||!std::isfinite(m->pose.pose.position.y)||!std::isfinite(m->twist.twist.linear.x)||!odom_clock.accept(stamp(m->header.stamp),now().seconds(),mono()))throw std::runtime_error("odom invalid");
        measured=std::abs(m->twist.twist.linear.x);odom_ok=true;
      }catch(const std::exception &){odom_ok=false;stop("odom_invalid");}
    });
    scan_sub=create_subscription<Scan>("/baseline/scan",rclcpp::SensorDataQoS().keep_last(1),[this](Scan::ConstSharedPtr m){
      scan_ok=false;
      if(m->header.frame_id=="laser"&&std::isfinite(m->angle_min)&&std::isfinite(m->angle_increment)&&m->angle_increment>0.&&m->range_min>=0.&&m->range_min<m->range_max&&std::isfinite(m->range_max)){
        double best=INFINITY;
        for(size_t i=0;i<m->ranges.size();++i){auto v=m->ranges[i];if(std::abs(m->angle_min+i*m->angle_increment)<=.35&&std::isfinite(v)&&v>=m->range_min&&v<=m->range_max)best=std::min(best,static_cast<double>(v));}
        if(std::isfinite(best)&&scan_clock.accept(stamp(m->header.stamp),now().seconds(),mono())){scan_ok=true;clearance=best;}
      }
      if(!scan_ok||clearance<.45)stop(scan_ok?"front_obstacle":"scan_invalid");
    });
    quality_sub=create_subscription<Bool>("/baseline/localization_valid",1,[this](Bool::ConstSharedPtr m){quality=m->data;quality_time=mono();if(!quality)stop("localization_invalid");});
    enable_sub=create_subscription<Bool>("/baseline/enable",1,[this](Bool::ConstSharedPtr m){enabled=m->data;enable_time=mono();if(!enabled)operator_stop();});
    watchdog=create_wall_timer(std::chrono::milliseconds(20),[this](){
      auto reason=unhealthy(mono(),now().seconds());
      if(!reason.empty())stop(reason);
      else if(!enabled||mono()-enable_time>.50)operator_stop();
      // Never renew a nonzero command on a timer; only a NEW accepted pose can.
    });
  }
};
int main(int argc,char **argv){
  const char *domain=std::getenv("ROS_DOMAIN_ID"),*local=std::getenv("ROS_LOCALHOST_ONLY");
  const char *permit=std::getenv("F1TENTH_GROUND_ROUTE_TEST");
  const bool offline=domain && std::string(domain)=="83";
  const bool ground=domain && std::string(domain)=="85" && permit &&
                   std::string(permit)=="bounded_25s_L1";
  if((!offline&&!ground)||!local||std::string(local)!="1"){
    std::fprintf(stderr,"REFUSED: requires domain 83 offline or explicit 25-second ground-route domain 85; localhost 1\n");return 2;}
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<Node>());}catch(const std::exception &e){std::fprintf(stderr,"%s\n",e.what());rclcpp::shutdown();return 2;}
  rclcpp::shutdown();return 0;
}
