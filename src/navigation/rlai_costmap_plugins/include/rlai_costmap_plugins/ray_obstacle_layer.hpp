// Copyright 2026 Robolabs AI. SPDX-License-Identifier: Apache-2.0
#ifndef RLAI_COSTMAP_PLUGINS__RAY_OBSTACLE_LAYER_HPP_
#define RLAI_COSTMAP_PLUGINS__RAY_OBSTACLE_LAYER_HPP_
#include <nav2_costmap_2d/obstacle_layer.hpp>

namespace rlai_costmap_plugins
{
// Inherit Nav2's observation buffers, TF, marking, footprint and lifecycle.
// Only replace the integer-endpoint free-space traversal.
class RayObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
protected:
  void raytraceFreespace(
    const nav2_costmap_2d::Observation & observation,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
};
}  // namespace rlai_costmap_plugins
#endif
