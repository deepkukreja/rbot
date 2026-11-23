// Copyright 2026 Robolabs AI. SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "rlai_costmap_plugins/ray_obstacle_layer.hpp"

template<class Layer> class Probe : public Layer
{
public:
  using Layer::raytraceFreespace;
};
using Tested = Probe<rlai_costmap_plugins::RayObstacleLayer>;
using Native = Probe<nav2_costmap_2d::ObstacleLayer>;

void require(bool ok, const char * message)
{
  if (!ok) {throw std::runtime_error(message);}
}

nav2_costmap_2d::Observation observation(
  double ox, double oy, float ex, float ey, double minimum=.25, double maximum=12.)
{
  nav2_costmap_2d::Observation result;
  result.origin_.x=ox; result.origin_.y=oy;
  result.raytrace_min_range_=minimum; result.raytrace_max_range_=maximum;
  auto & cloud=*result.cloud_;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1,"xyz"); modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"), y(cloud,"y"), z(cloud,"z");
  *x=ex; *y=ey; *z=.35f;
  return result;
}

void real_fractional_ray_clears_crossed_cells()
{
  // Recorded scan at t=31.219 after box deletion. This measured ray spends
  // >5 cm inside (247,209). Literal cell/segment values are independent of
  // the traversal under test; truncating the two endpoints misses this cell.
  auto obs=observation(4.23618608306807,.0391345892234837,
    -10.913914680480957f,-2.9929187297821045f);
  Native native; Tested layer;
  for (nav2_costmap_2d::ObstacleLayer * p : {static_cast<nav2_costmap_2d::ObstacleLayer *>(&native),
      static_cast<nav2_costmap_2d::ObstacleLayer *>(&layer)}) {
    p->resizeMap(399,400,.05000000074505806,-10.978,-11.024);
    p->setCost(247,209,nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  double a=1e100,b=1e100,c=-1e100,d=-1e100;
  native.raytraceFreespace(obs,&a,&b,&c,&d);
  std::cout << "native retained recorded cell: " <<
    (native.getCost(247,209)==nav2_costmap_2d::LETHAL_OBSTACLE) << '\n';
  a=b=1e100; c=d=-1e100;
  layer.raytraceFreespace(obs,&a,&b,&c,&d);
  require(layer.getCost(247,209)==nav2_costmap_2d::FREE_SPACE,
    "recorded measured ray must clear positively intersected cell (247,209)");
  std::cout << "bounds: " << a << " " << b << " " << c << " " << d << std::endl;
  require(a<1.372 && c>1.422 && b<-.574 && d>-.524,
    "update bounds must cover cleared recorded cell");
}

void range_limits_and_no_beam_widening()
{
  Tested layer; layer.resizeMap(100,40,.1,0.,0.);
  std::fill_n(layer.getCharMap(),4000,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto obs=observation(1.025,1.025,4.025f,1.025f,.25,2.);
  double a=1e100,b=1e100,c=-1e100,d=-1e100;
  layer.raytraceFreespace(obs,&a,&b,&c,&d);
  require(layer.getCost(11,10)==254,"minimum range must retain near-sensor cells");
  require(layer.getCost(12,10)==0 && layer.getCost(30,10)==0,
    "positive free segments within minimum/maximum range must clear");
  require(layer.getCost(31,10)==254,"maximum range must retain cells beyond range");
  require(layer.getCost(20,9)==254 && layer.getCost(20,11)==254,
    "a measured beam must not clear adjacent unobserved rows");
}

void measured_return_keeps_occluded_space()
{
  Tested layer; layer.resizeMap(100,40,.1,0.,0.);
  std::fill_n(layer.getCharMap(),4000,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto obs=observation(1.025,1.025,4.025f,1.025f);
  double a=1e100,b=1e100,c=-1e100,d=-1e100;
  layer.raytraceFreespace(obs,&a,&b,&c,&d);
  require(layer.getCost(39,10)==0,"observed free interval before return must clear");
  require(layer.getCost(41,10)==254 && layer.getCost(50,10)==254,
    "obstacle and shadow cells beyond a measured endpoint must remain marked");
  // Nav2's inherited marking pass restores the measured-return cell itself.
}

void clipped_endpoint_and_invalid_segments()
{
  Tested layer; layer.resizeMap(100,40,.1,0.,0.);
  std::fill_n(layer.getCharMap(),4000,nav2_costmap_2d::LETHAL_OBSTACLE);
  auto obs=observation(1.025,1.025,20.f,1.025f);
  double a=1e100,b=1e100,c=-1e100,d=-1e100;
  layer.raytraceFreespace(obs,&a,&b,&c,&d);
  require(layer.getCost(99,10)==0,"off-map endpoint must clip to grid boundary");
  auto invalid=observation(1.025,2.025,std::numeric_limits<float>::quiet_NaN(),2.025f);
  a=b=1e100; c=d=-1e100;
  layer.raytraceFreespace(invalid,&a,&b,&c,&d);
  require(layer.getCost(20,20)==254 && a==1e100 && c==-1e100,
    "non-finite endpoint must not clear or expand bounds");
  auto zero=observation(1.,2.,1.f,2.f);
  layer.raytraceFreespace(zero,&a,&b,&c,&d);
  require(layer.getCost(10,20)==254,"zero-length ray must not clear");
  auto reversed=observation(1.,2.,4.f,2.f,3.,2.);
  layer.raytraceFreespace(reversed,&a,&b,&c,&d);
  require(layer.getCost(25,20)==254,"empty configured range must not clear");
  auto outside=observation(-1.,3.,5.f,3.f);
  layer.raytraceFreespace(outside,&a,&b,&c,&d);
  require(layer.getCost(20,30)==254,"outside-grid origin must preserve Nav2 rejection");
}

int main()
{
  try {
    real_fractional_ray_clears_crossed_cells();
    range_limits_and_no_beam_widening();
    measured_return_keeps_occluded_space();
    clipped_endpoint_and_invalid_segments();
    std::cout << "4 obstacle-ray regression cases passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; return 1;
  }
  return 0;
}
