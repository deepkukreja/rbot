// Copyright 2026 Robolabs AI. SPDX-License-Identifier: Apache-2.0
#include <cstdlib>
#include <iostream>
#include <random>
#include <set>
#include "rlai_costmap_plugins/grid_ray.hpp"
using namespace rlai_costmap_plugins;
using Cells = std::set<std::array<int, 3>>;

// Independent slab-intersection oracle: enumerate every cell's bounds rather
// than following the traversal implementation or rounding a ray to cell centres.
Cells reference(const Vec3 & start, const Vec3 & end, const GridSize & size)
{
  Cells result;
  if (start == end) {return result;}
  for (int x = 0; x < size[0]; ++x) {
    for (int y = 0; y < size[1]; ++y) {
      for (int z = 0; z < size[2]; ++z) {
        const GridSize cell{x, y, z}; double lo = 0.0, hi = 1.0; bool valid = true;
        for (int axis = 0; axis < 3; ++axis) {
          const double d = end[axis] - start[axis];
          if (d == 0.0) {
            if (start[axis] < cell[axis] || start[axis] >= cell[axis] + 1) {valid = false;}
          } else {
            const double a = (cell[axis] - start[axis]) / d;
            const double b = (cell[axis] + 1 - start[axis]) / d;
            lo = std::max(lo, std::min(a, b)); hi = std::min(hi, std::max(a, b));
          }
        }
        if (valid && hi - lo > 1e-12) {result.insert(cell);}
      }
    }
  }
  return result;
}

void check(const Vec3 & start, const Vec3 & end, const GridSize & size)
{
  Cells actual;
  traceGridRay(start, end, size, [&actual](int x, int y, int z) {
    if (!actual.insert({x, y, z}).second) {std::abort();}
  });
  if (actual != reference(start, end, size)) {
    std::cerr << "Ray traversal differs from segment/AABB intersections\n";
    std::exit(1);
  }
}

int main()
{
  const GridSize size{8, 8, 8};
  for (const auto & start : {Vec3{0, 0, 0}, Vec3{8, 8, 8}, Vec3{4, 4, 4}, Vec3{-1, 4, 4}}) {
    for (const auto & end : {Vec3{8, 8, 8}, Vec3{0, 0, 0}, Vec3{4, 4, 4}, Vec3{4, 4, 8}, Vec3{9, 4, 4}}) {
      check(start, end, size);
    }
  }
  // Axis-aligned, negative direction, clipped, short segment, and zero length.
  check({1.2, 3.5, 2.9}, {1.8, 3.5, 2.9}, size);
  check({7.9, 4.2, 6.1}, {0.1, 4.2, 6.1}, size);
  check({-2, .5, .5}, {12, .5, .5}, size);
  check({2.2, 3.3, 4.4}, {2.2, 3.3, 4.4}, size);
  std::mt19937 random(20731); std::uniform_real_distribution<double> dist(-2., 10.);
  for (int trial = 0; trial < 2000; ++trial) {
    Vec3 start{dist(random), dist(random), dist(random)};
    Vec3 end{dist(random), dist(random), dist(random)};
    check(start, end, size);
  }
  // This measured finite ray must never clear a cell behind its endpoint.
  Cells short_ray;
  traceGridRay({.5, .5, .5}, {2.8, .5, .5}, size, [&short_ray](int x, int y, int z) {short_ray.insert({x, y, z});});
  if (short_ray != Cells{{0,0,0}, {1,0,0}, {2,0,0}}) {return 1;}
  std::cout << "2025 geometric ray cases passed, including finite endpoint protection\n";
}
