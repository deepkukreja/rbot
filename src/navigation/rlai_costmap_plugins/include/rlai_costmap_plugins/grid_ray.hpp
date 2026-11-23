// Copyright 2026 Robolabs AI. SPDX-License-Identifier: Apache-2.0
#ifndef RLAI_COSTMAP_PLUGINS__GRID_RAY_HPP_
#define RLAI_COSTMAP_PLUGINS__GRID_RAY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace rlai_costmap_plugins
{
using Vec3 = std::array<double, 3>;
using GridSize = std::array<int, 3>;

// Clip a finite measured segment to the grid. Merely touching a cell at one
// point is not evidence of a free-space interval within that cell.
inline std::optional<std::pair<double, double>> clipGridRay(
  const Vec3 & start, const Vec3 & end, const GridSize & size)
{
  double enter = 0.0, leave = 1.0;
  bool nonzero = false;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(start[axis]) || !std::isfinite(end[axis]) || size[axis] <= 0) {
      return std::nullopt;
    }
    const double delta = end[axis] - start[axis];
    nonzero = nonzero || delta != 0.0;
    if (delta == 0.0) {
      if (start[axis] < 0.0 || start[axis] >= size[axis]) {return std::nullopt;}
      continue;
    }
    double a = -start[axis] / delta;
    double b = (size[axis] - start[axis]) / delta;
    if (a > b) {std::swap(a, b);}
    enter = std::max(enter, a);
    leave = std::min(leave, b);
  }
  if (!nonzero || leave - enter <= 1e-12) {return std::nullopt;}
  return std::make_pair(enter, leave);
}

// Amanatides-Woo traversal retains the fractional sensor and endpoint positions.
// Visit exactly the grid cells crossed for a positive segment length, without
// enlarging a ray, extending it past its measured endpoint, or interpolating beams.
template<typename Visitor>
void traceGridRay(const Vec3 & start, const Vec3 & end, const GridSize & size, Visitor visit)
{
  const auto interval = clipGridRay(start, end, size);
  if (!interval) {return;}
  const double enter = interval->first, leave = interval->second;
  const double probe = enter + std::min(1e-10, (leave - enter) * 0.5);
  GridSize cell{}, step{};
  Vec3 next{}, increment{};
  for (int axis = 0; axis < 3; ++axis) {
    const double delta = end[axis] - start[axis];
    cell[axis] = std::clamp(
      static_cast<int>(std::floor(start[axis] + delta * probe)), 0, size[axis] - 1);
    if (delta == 0.0) {
      step[axis] = 0;
      next[axis] = increment[axis] = std::numeric_limits<double>::infinity();
    } else {
      step[axis] = delta > 0.0 ? 1 : -1;
      const int boundary = cell[axis] + (step[axis] > 0 ? 1 : 0);
      next[axis] = (boundary - start[axis]) / delta;
      increment[axis] = 1.0 / std::abs(delta);
    }
  }
  double current = enter;
  const int limit = size[0] + size[1] + size[2] + 3;
  for (int iteration = 0; iteration < limit; ++iteration) {
    const double crossing = std::min({next[0], next[1], next[2], leave});
    if (crossing - current > 1e-12) {visit(cell[0], cell[1], cell[2]);}
    if (crossing >= leave - 1e-12) {break;}
    for (int axis = 0; axis < 3; ++axis) {
      if (next[axis] <= crossing + 1e-12) {
        cell[axis] += step[axis];
        next[axis] += increment[axis];
      }
      if (cell[axis] < 0 || cell[axis] >= size[axis]) {return;}
    }
    current = crossing;
  }
}
}  // namespace rlai_costmap_plugins
#endif
