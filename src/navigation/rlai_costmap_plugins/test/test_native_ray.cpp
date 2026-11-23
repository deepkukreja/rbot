// Copyright 2026 Robolabs AI. SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <vector>
#include <nav2_voxel_grid/voxel_grid.hpp>
#include "rlai_costmap_plugins/grid_ray.hpp"

int main()
{
  using namespace rlai_costmap_plugins;
  // A real ray from the preserved low_3d_raw_clear bag at simulation t=48.3.
  // The ray crosses over 5 cm of the marked voxel, beyond the deleted box.
  const Vec3 sensor{3.9653558912687656, -.05607222975753153, .3625};
  const Vec3 endpoint{1.0263970663225885, -.3363668146283638, .000003695767889777546};
  const Vec3 origin{-10.978, -11.024, 0}, resolution{.05, .05, .1};
  Vec3 begin{}, end{};
  for (int axis = 0; axis < 3; ++axis) {
    begin[axis] = (sensor[axis] - origin[axis]) / resolution[axis];
    end[axis] = (endpoint[axis] - origin[axis]) / resolution[axis];
  }
  nav2_voxel_grid::VoxelGrid native(399, 400, 16);
  native.markVoxel(252, 214, 0);
  std::vector<unsigned char> costmap(399 * 400, 0);
  native.clearVoxelLineInMap(begin[0], begin[1], begin[2], end[0], end[1], end[2], costmap.data(), 15, 0);
  const bool native_missed = native.getVoxel(252, 214, 0) == nav2_voxel_grid::MARKED;
  native.markVoxel(252, 214, 0);
  traceGridRay(begin, end, {399, 400, 16}, [&native](int x, int y, int z) {native.clearVoxel(x, y, z);});
  if (native.getVoxel(252, 214, 0) != nav2_voxel_grid::FREE) {
    std::cerr << "Measured-ray traversal failed to clear the intersected voxel\n";
    return 1;
  }
  // Clearing stops before the measured return. A farther obstacle stays marked.
  native.markVoxel(10, 10, 3); native.markVoxel(14, 10, 3);
  traceGridRay({8.5, 10.5, 3.5}, {12.2, 10.5, 3.5}, {399,400,16},
    [&native](int x, int y, int z) {native.clearVoxel(x, y, z);});
  if (native.getVoxel(10, 10, 3) != nav2_voxel_grid::FREE ||
    native.getVoxel(14, 10, 3) != nav2_voxel_grid::MARKED) {return 1;}
  std::cout << (native_missed ? "Installed C++ counterexample confirmed; " : "Upstream native traversal already clears this case; ")
            << "corrected traversal and occlusion endpoint passed\n";
}
