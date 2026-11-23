// Copyright 2026 Robolabs AI. SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "rlai_costmap_plugins/grid_ray.hpp"
#include "rlai_costmap_plugins/ray_obstacle_layer.hpp"

namespace rlai_costmap_plugins
{
void RayObstacleLayer::raytraceFreespace(
  const nav2_costmap_2d::Observation & observation,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  const double ox=observation.origin_.x, oy=observation.origin_.y;
  const double minimum=std::max(0.0, observation.raytrace_min_range_);
  const double maximum=observation.raytrace_max_range_;
  if (!std::isfinite(ox) || !std::isfinite(oy) ||
    !std::isfinite(observation.raytrace_min_range_) || !std::isfinite(maximum) ||
    maximum <= minimum) {return;}
  unsigned int mx, my;
  if (!worldToMap(ox, oy, mx, my)) {
    RCLCPP_WARN(logger_, "2D clearing sensor origin is outside the costmap");
    return;
  }
  const GridSize size{static_cast<int>(size_x_), static_cast<int>(size_y_), 1};
  const auto & cloud=*observation.cloud_;
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y");
  for (; x != x.end(); ++x, ++y) {
    const double dx=*x-ox, dy=*y-oy;
    const double distance=std::hypot(dx, dy);
    const double usable=std::min(distance, maximum);
    if (!std::isfinite(distance) || distance <= 0.0 || usable <= minimum) {continue;}
    // Keep fractional positions through range clipping and grid traversal.
    // Z=.5 embeds a 2D ray in the existing positive-length 3D cell walker.
    // Match ObstacleLayer's measured endpoint; its marking pass restores the
    // return cell. Do not extend the ray or clear any cell behind the return.
    const Vec3 begin{(ox+dx*minimum/distance-origin_x_)/resolution_,
      (oy+dy*minimum/distance-origin_y_)/resolution_, .5};
    const Vec3 end{(ox+dx*usable/distance-origin_x_)/resolution_,
      (oy+dy*usable/distance-origin_y_)/resolution_, .5};
    const auto interval=clipGridRay(begin, end, size);
    if (!interval) {continue;}
    touch(ox, oy, min_x, min_y, max_x, max_y);
    traceGridRay(begin, end, size, [this](int ix, int iy, int) {
      costmap_[getIndex(ix, iy)]=nav2_costmap_2d::FREE_SPACE;
    });
    const double ex=origin_x_+resolution_*(begin[0]+(end[0]-begin[0])*interval->second);
    const double ey=origin_y_+resolution_*(begin[1]+(end[1]-begin[1])*interval->second);
    updateRaytraceBounds(ox, oy, ex, ey, maximum, minimum, min_x, min_y, max_x, max_y);
  }
}
}  // namespace rlai_costmap_plugins
PLUGINLIB_EXPORT_CLASS(rlai_costmap_plugins::RayObstacleLayer, nav2_costmap_2d::Layer)
