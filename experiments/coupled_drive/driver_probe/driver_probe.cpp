// SPDX-License-Identifier: Apache-2.0
// Native driver callbacks with fake joint interfaces, never a robot publisher.
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <queue>
#include <vector>
#include "diff_drive_controller/diff_drive_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

class Driver : public diff_drive_controller::DiffDriveController {
public:
  void reference(double v,double w) {reference_interfaces_[0]=v;reference_interfaces_[1]=w;}
  void seed(double v,double w) {
    previous_two_commands_={};previous_two_commands_.push({v,w});previous_two_commands_.push({v,w});
  }
  auto history() {return previous_two_commands_.back();}
};

int main(int argc,char **argv) {
  const bool legacy=argc>1&&std::string(argv[1])=="legacy";
  const bool reject_jerk=argc>1&&std::string(argv[1])=="jerk";
  rclcpp::init(argc,argv);
  double position[2]={0.,0.},command[2]={0.,0.};
  hardware_interface::StateInterface lp("left","position",&position[0]),rp("right","position",&position[1]);
  hardware_interface::CommandInterface lc("left","velocity",&command[0]),rc("right","velocity",&command[1]);
  auto driver=std::make_unique<Driver>();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("left_wheel_names",std::vector<std::string>{"left"}),
    rclcpp::Parameter("right_wheel_names",std::vector<std::string>{"right"}),
    rclcpp::Parameter("wheel_separation",.35),rclcpp::Parameter("wheel_separation_multiplier",.886),
    rclcpp::Parameter("wheel_radius",.0625),rclcpp::Parameter("coupled_wheel_velocity_limit",legacy?0.:5.),
    rclcpp::Parameter("linear.x.max_velocity",.5),rclcpp::Parameter("linear.x.min_velocity",-.35),
    rclcpp::Parameter("linear.x.max_acceleration",1.),rclcpp::Parameter("linear.x.min_acceleration",-1.),
    rclcpp::Parameter("linear.x.has_jerk_limits",reject_jerk),
    rclcpp::Parameter("angular.z.max_velocity",1.9),rclcpp::Parameter("angular.z.min_velocity",-1.9),
    rclcpp::Parameter("angular.z.max_acceleration",3.2),rclcpp::Parameter("angular.z.min_acceleration",-3.2),
    rclcpp::Parameter("angular.z.has_jerk_limits",false),rclcpp::Parameter("publish_limited_velocity",true)});
  if(reject_jerk) {
    auto overrides=options.parameter_overrides();
    overrides.emplace_back("linear.x.max_jerk",1.);overrides.emplace_back("linear.x.min_jerk",-1.);
    options.parameter_overrides(overrides);
  }
  controller_interface::ControllerInterfaceParams params;
  params.controller_name="coupled_driver_probe";params.robot_description="";params.update_rate=100;params.node_options=options;
  if(driver->init(params)!=controller_interface::return_type::OK) return 2;
  const auto configured=driver->on_configure(rclcpp_lifecycle::State());
  if(reject_jerk) {
    const bool rejected=configured==controller_interface::CallbackReturn::ERROR;
    std::cout<<"{\"jerk_rejected\":"<<(rejected?"true":"false")<<"}\n";
    driver.reset();rclcpp::shutdown();return rejected?0:1;
  }
  if(configured!=controller_interface::CallbackReturn::SUCCESS) return 2;
  std::vector<hardware_interface::LoanedStateInterface> states;states.emplace_back(lp);states.emplace_back(rp);
  std::vector<hardware_interface::LoanedCommandInterface> commands;commands.emplace_back(lc);commands.emplace_back(rc);
  driver->assign_interfaces(std::move(commands),std::move(states));
  if(driver->on_activate(rclcpp_lifecycle::State())!=controller_interface::CallbackReturn::SUCCESS) return 2;
  double peak=0.,max_v_delta=0.,max_w_delta=0.,history_error=0.;int ticks=0;
  auto tick=[&](double v,double w,double &old_v,double &old_w) {
    driver->reference(v,w);++ticks;
    const auto time=rclcpp::Time(1'000'000'000LL+ticks*10'000'000LL,RCL_ROS_TIME);
    if(driver->update_and_write_commands(time,rclcpp::Duration::from_seconds(.01))!=controller_interface::return_type::OK) throw std::runtime_error("Driver update failed");
    const double out_v=.0625*(command[0]+command[1])/2;
    const double out_w=.0625*(command[1]-command[0])/(.35*.886);
    peak=std::max(peak,std::max(std::abs(command[0]),std::abs(command[1])));
    max_v_delta=std::max(max_v_delta,std::abs(out_v-old_v));max_w_delta=std::max(max_w_delta,std::abs(out_w-old_w));
    const auto history=driver->history();history_error=std::max(history_error,std::max(std::abs(out_v-history[0]),std::abs(out_w-history[1])));
    old_v=out_v;old_w=out_w;
  };
  double v=0.,w=1.9;driver->seed(v,w);
  for(int i=0;i<100;++i) tick(.3125,0.,v,w);
  const bool transition_reached=std::abs(v-.3125)<1e-8&&std::abs(w)<1e-8;
  driver->seed(0.,0.);v=0.;w=0.;
  for(int i=0;i<120;++i) tick(.3,1.,v,w);
  const double mixed_v=v,mixed_w=w;
  for(int i=0;i<150;++i) tick(0.,0.,v,w);
  const bool bounded=peak<=5.&&max_v_delta<=.010000001&&max_w_delta<=.032000001&&history_error<1e-12&&transition_reached&&std::abs(v)<1e-9&&std::abs(w)<1e-9&&std::abs(mixed_w/mixed_v-1./.3)<1e-9;
  const bool passed=legacy ?
    (std::abs(peak-7.2808)<1e-10&&std::abs(mixed_v-.3)<1e-10&&std::abs(mixed_w-1.)<1e-10&&history_error<1e-12) : bounded;
  std::cout<<std::setprecision(17)<<"{\"passed\":"<<(passed?"true":"false")<<",\"peak_wheel_rad_s\":"<<peak
    <<",\"max_linear_delta\":"<<max_v_delta<<",\"max_angular_delta\":"<<max_w_delta
    <<",\"history_error\":"<<history_error<<",\"transition_reached\":"<<(transition_reached?"true":"false")
    <<",\"mixed_v\":"<<mixed_v<<",\"mixed_w\":"<<mixed_w<<",\"ticks\":"<<ticks<<"}\n";
  driver.reset();rclcpp::shutdown();return passed?0:1;
}
