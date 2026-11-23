// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include "nav2_mppi_controller/optimizer.hpp"

class Probe : public mppi::Optimizer {
public:
  Probe(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map, int axis, float target,
    bool shifted, float ratio = 1.f, bool invalid = false, bool constant = false)
  {
    costmap_ros_ = map;
    settings_.time_steps = 56;settings_.batch_size = 1;settings_.model_dt = .05f;
    // Isolate the real native finalization from stochastic optimization.
    settings_.iteration_count = 0;settings_.shift_control_sequence = shifted;
    settings_.base_constraints = {.5f, -.35f, 0.f, 1.9f, 3.f, -3.f, 3.f, -3.f, 3.5f};
    settings_.constraints = settings_.base_constraints;
    setMotionModel("DiffDrive");
    if(ratio != 1.f) {setSpeedLimit(ratio * 100., true);}
    control_sequence_.reset(56);state_.reset(1, 56);costs_ = xt::zeros<float>({1});
    const float step = constant ? 0.f : (target < 0 ? -.1f : .1f);
    control_history_.fill({0.f, 0.f, 0.f});
    for(int i = 0; i < 4; ++i) {
      if(axis == 0) {control_history_[i].vx = target - step * (3 - i);} else {
        control_history_[i].wz = target - step * (3 - i);
      }
    }
    if(axis == 0) {control_sequence_.vx.fill(target);} else {control_sequence_.wz.fill(target);}
    if(invalid) {control_sequence_.wz(1) = std::numeric_limits<float>::quiet_NaN();}
  }
  auto run()
  {
    geometry_msgs::msg::PoseStamped pose;pose.pose.orientation.w = 1.;
    nav_msgs::msg::Path path;path.header.frame_id = "map";path.header.stamp.sec = 7;
    path.poses = {pose, pose};
    geometry_msgs::msg::Twist speed;speed.linear.x = control_history_[3].vx;
    speed.angular.z = control_history_[3].wz;
    return evalControl(pose, speed, path, pose.pose, nullptr);
  }
  auto history() {return control_history_[3];}
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("bounds_probe_costmap");
  struct Case{const char *name;int axis;float target;bool shift;float ratio;bool invalid;
    bool constant = false;};
  const std::array<Case, 10> cases{{
    {"turn_positive_shift", 2, 1.9f, true, 1.f, false},
    {"turn_negative_shift", 2, -1.9f, true, 1.f, false},
    {"forward_shift", 0, .5f, true, 1.f, false}, {"reverse_shift", 0, -.35f, true, 1.f, false},
    {"turn_positive_unshifted", 2, 1.9f, false, 1.f, false},
    {"reverse_unshifted", 0, -.35f, false, 1.f, false},
    {"reduced_speed_limit", 2, .95f, true, .5f, false},
    {"inside_limit", 2, .5f, true, 1.f, false, true},
    {"nonfinite_sequence", 2, 1.f, true, 1.f, true},
    {"nonfinite_sequence_unshifted", 2, 1.f, false, 1.f, true}}};
  int failures = 0;std::cout << std::setprecision(12) << "[";bool first = true;
  for(const auto & c:cases) {
    Probe probe(map, c.axis, c.target, c.shift, c.ratio, c.invalid, c.constant);
    bool threw = false, passed = false;double value = 0, history = 0;
    try {
      const auto command = probe.run();
      value = c.axis == 0 ? command.twist.linear.x : command.twist.angular.z;
      history = c.axis == 0 ? probe.history().vx : probe.history().wz;
      const double lower = c.axis == 0 ? -.35 * c.ratio : -1.9 * c.ratio;
      const double upper = c.axis == 0 ? .5 * c.ratio : 1.9 * c.ratio;
      passed = !c.invalid && std::isfinite(value) && value >= lower && value <= upper &&
        std::isfinite(history) && std::abs(history - value) < 1e-7 && command.header.stamp.sec == 7;
      if(std::string(c.name) == "inside_limit") {passed = passed && std::abs(value - .5) < 1e-6;}
    } catch(const nav2_core::NoValidControl &) {
      threw = true;passed = c.invalid;
    }
    if(!passed) {++failures;}
    if(!first) {std::cout << ",";} first = false;
    std::cout << "{\"name\":\"" << c.name << "\",\"value\":";
    if(std::isfinite(value)) {std::cout << value;} else {std::cout << "null";}
    std::cout << ",\"history\":";if(std::isfinite(history)) {std::cout << history;} else {
      std::cout << "null";
    }
    std::cout << ",\"threw\":" << (threw ? "true" : "false") << ",\"passed\":" <<
      (passed ? "true" : "false") << "}";
  }
  std::cout << "]\n";map.reset();rclcpp::shutdown();return failures ? 1 : 0;
}
