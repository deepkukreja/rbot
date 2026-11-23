// Copyright 2026 Robolabs AI. SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include <nav2_costmap_2d/voxel_layer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "rlai_costmap_plugins/grid_ray.hpp"

namespace rlai_costmap_plugins
{
// Keep Nav2's observation buffers, TF, marking, footprint handling, costmap
// combination and lifecycle. Only replace integer-endpoint clearing traversal.
class RayVoxelLayer : public nav2_costmap_2d::VoxelLayer
{
protected:
  void raytraceFreespace(
    const nav2_costmap_2d::Observation & observation,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    const Vec3 origin{observation.origin_.x, observation.origin_.y, observation.origin_.z};
    double gx, gy, gz;
    if (!worldToMap3DFloat(origin[0], origin[1], origin[2], gx, gy, gz)) {
      RCLCPP_WARN(logger_, "3D clearing sensor origin is outside the voxel grid");
      return;
    }
    const GridSize size{static_cast<int>(size_x_), static_cast<int>(size_y_), size_z_};
    const Vec3 grid_origin{origin_x_, origin_y_, origin_z_};
    const Vec3 resolution{resolution_, resolution_, z_resolution_};
    const bool debug = clearing_endpoints_pub_->get_subscription_count() > 0;
    std::vector<std::array<float, 3>> debug_points;
    const auto & cloud = *observation.cloud_;
    // Hundreds of rays can cross one XY column. Clear every intersected voxel,
    // but classify each touched column once after the clearing batch. Clearing
    // only removes occupancy, so this preserves the per-visit final result.
    std::vector<unsigned char> touched_mask(size_x_ * size_y_, 0);
    std::vector<unsigned int> touched;
    touched.reserve(std::min(size_x_ * size_y_, cloud.width * cloud.height * 4));
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      Vec3 direction{*x - origin[0], *y - origin[1], *z - origin[2]};
      const double distance = std::hypot(std::hypot(direction[0], direction[1]), direction[2]);
      // Match Nav2's two-horizontal-cell endpoint margin. Range is in metres;
      // no cells behind a measured return or beyond the range limit are cleared.
      const double usable = std::min(distance - 2.0 * resolution_, observation.raytrace_max_range_);
      const double minimum = std::max(0.0, observation.raytrace_min_range_);
      if (!std::isfinite(distance) || usable <= minimum || distance <= 0.0) {continue;}
      Vec3 begin{}, end{};
      for (int axis = 0; axis < 3; ++axis) {
        direction[axis] /= distance;
        begin[axis] = (origin[axis] + direction[axis] * minimum - grid_origin[axis]) / resolution[axis];
        end[axis] = (origin[axis] + direction[axis] * usable - grid_origin[axis]) / resolution[axis];
      }
      const auto interval = clipGridRay(begin, end, size);
      if (!interval) {continue;}
      traceGridRay(begin, end, size, [this, &touched, &touched_mask](int ix, int iy, int iz) {
        voxel_grid_.clearVoxel(ix, iy, iz);
        const auto index = getIndex(ix, iy);
        if (!touched_mask[index]) {
          touched_mask[index] = 1;
          touched.push_back(index);
        }
      });
      const double ex = grid_origin[0] + resolution[0] * (begin[0] + (end[0] - begin[0]) * interval->second);
      const double ey = grid_origin[1] + resolution[1] * (begin[1] + (end[1] - begin[1]) * interval->second);
      updateRaytraceBounds(
        origin[0], origin[1], ex, ey, observation.raytrace_max_range_,
        observation.raytrace_min_range_, min_x, min_y, max_x, max_y);
      if (debug) {
        const double ez = grid_origin[2] + resolution[2] * (begin[2] + (end[2] - begin[2]) * interval->second);
        debug_points.push_back({static_cast<float>(ex), static_cast<float>(ey), static_cast<float>(ez)});
      }
    }
    for (const auto index : touched) {
      const auto state = voxel_grid_.getVoxelColumn(
        index % size_x_, index / size_x_, unknown_threshold_, mark_threshold_);
      if (state == nav2_voxel_grid::FREE) {
        costmap_[index] = nav2_costmap_2d::FREE_SPACE;
      } else if (state == nav2_voxel_grid::UNKNOWN) {
        costmap_[index] = nav2_costmap_2d::NO_INFORMATION;
      }
    }
    if (debug) {
      sensor_msgs::msg::PointCloud2 message;
      message.header = cloud.header;
      message.header.frame_id = global_frame_;
      sensor_msgs::PointCloud2Modifier modifier(message);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(debug_points.size());
      sensor_msgs::PointCloud2Iterator<float> ox(message, "x"), oy(message, "y"), oz(message, "z");
      for (const auto & point : debug_points) {
        *ox = point[0]; *oy = point[1]; *oz = point[2]; ++ox; ++oy; ++oz;
      }
      clearing_endpoints_pub_->publish(std::move(message));
    }
  }
};
}  // namespace rlai_costmap_plugins
PLUGINLIB_EXPORT_CLASS(rlai_costmap_plugins::RayVoxelLayer, nav2_costmap_2d::Layer)
