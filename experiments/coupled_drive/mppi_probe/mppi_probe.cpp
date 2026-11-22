// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include "nav2_mppi_controller/optimizer.hpp"

class Probe : public mppi::Optimizer {
public:
  Probe(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map,bool transition,bool shifted,float ratio=1.f) {
    costmap_ros_=map;settings_.time_steps=56;settings_.batch_size=1;settings_.model_dt=.05f;
    settings_.iteration_count=0;settings_.shift_control_sequence=shifted;
    settings_.base_constraints={.5f,-.35f,0.f,1.9f,1.f,-1.f,1.f,-1.f,3.2f};
    settings_.constraints=settings_.base_constraints;setMotionModel("DiffDrive");
#ifdef RLAI_COUPLED_DIFF_DRIVE
    dynamic_cast<mppi::DiffDriveMotionModel*>(motion_model_.get())->configureWheelLimits(.3101,.0625,.0625,5.);
#endif
    if(ratio<1.f) setSpeedLimit(ratio*100.,true);
    control_sequence_.reset(56);state_.reset(1,56);costs_=xt::zeros<float>({1});
    control_sequence_.vx.fill(transition?.3125f:.3f);control_sequence_.wz.fill(transition?0.f:1.f);
    control_history_.fill({transition?.3125f:.3f,0.f,transition?0.f:1.f});
    initial_v=0.f;initial_w=transition?1.9f:0.f;
  }
  float first_v=0.f,first_w=0.f;bool reached=false;
  double rollout() {
    state_.vx.fill(0);state_.wz.fill(0);state_.vx(0,0)=initial_v;state_.wz(0,0)=initial_w;
    state_.cvx.fill(initial_w? .3125f:.3f);state_.cwz.fill(initial_w?0.f:1.f);
    motion_model_->predict(state_);double peak=0;
    for(unsigned int j=1;j<56;++j) peak=std::max(peak,wheel(state_.vx(0,j),state_.wz(0,j)));
    first_v=state_.vx(0,1);first_w=state_.wz(0,1);
    double target_v=std::min(double(initial_w?.3125f:.3f),double(settings_.constraints.vx_max));
    double target_w=std::min(double(initial_w?0.f:1.f),double(settings_.constraints.wz));
    const double factor=std::min(1.,5./std::max(1e-12,wheel(target_v,target_w)));
    target_v*=factor;target_w*=factor;
    reached=std::abs(state_.vx(0,55)-target_v)<1e-5&&std::abs(state_.wz(0,55)-target_w)<1e-5;
    return peak;
  }
  bool rejectsInvalidRollout() {
    state_.vx.fill(0);state_.wz.fill(0);state_.cvx.fill(.3f);state_.cwz.fill(1.f);
    state_.cvx(0,1)=std::numeric_limits<float>::quiet_NaN();
    try {motion_model_->predict(state_);} catch(const nav2_core::NoValidControl&) {return true;}
    return false;
  }
  auto command() {
    geometry_msgs::msg::PoseStamped pose;pose.pose.orientation.w=1.;nav_msgs::msg::Path path;
    path.header.frame_id="map";path.header.stamp.sec=3;path.poses={pose,pose};
    geometry_msgs::msg::Twist speed;speed.linear.x=initial_v;speed.angular.z=initial_w;
    return evalControl(pose,speed,path,pose.pose,nullptr);
  }
  auto history(){return control_history_[3];}
  static double wheel(double v,double w){return std::max(std::abs(v-.3101*w/2),std::abs(v+.3101*w/2))/.0625;}
  float initial_v,initial_w;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("coupled_probe_costmap");
  bool passed=true;double maximum=0.,history_error=0.;int cases=0;
  for(int kind:{0,1,2})for(bool shifted:{false,true}) {
    const bool transition=kind==1;const float ratio=kind==2?.5f:1.f;
    Probe p(map,transition,shifted,ratio);const double predicted=p.rollout();const auto c=p.command();const auto h=p.history();
    const double wheel=Probe::wheel(c.twist.linear.x,c.twist.angular.z);
    maximum=std::max(maximum,std::max(predicted,wheel));history_error=std::max(history_error,std::max(std::abs(h.vx-c.twist.linear.x),std::abs(h.wz-c.twist.angular.z)));
    passed=passed&&p.reached&&std::abs(c.twist.linear.x-p.first_v)<1e-6&&std::abs(c.twist.angular.z-p.first_w)<1e-6&&std::abs(c.twist.linear.x)<=.5*ratio+1e-7&&std::abs(c.twist.angular.z)<=1.9*ratio+1e-7&&predicted<=5.&&wheel<=5.&&std::abs(c.twist.linear.x-p.initial_v)<=.050001&&std::abs(c.twist.angular.z-p.initial_w)<=.160001;
    ++cases;
  }
  Probe invalid(map,false,false);const bool rejected=invalid.rejectsInvalidRollout();
  passed=passed&&history_error<1e-7&&rejected;
  std::cout<<std::setprecision(17)<<"{\"passed\":"<<(passed?"true":"false")<<",\"cases\":"<<cases<<",\"peak_wheel\":"<<maximum<<",\"invalid_rollout_rejected\":"<<(rejected?"true":"false")<<",\"history_error\":"<<history_error<<"}\n";
  map.reset();rclcpp::shutdown();return passed?0:1;
}
