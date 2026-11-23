cmake_minimum_required(VERSION 3.8)
project(rlai_costmap_plugins)

# Ray clearing runs while Nav2 holds the costmap mutex. An unspecified build
# type would compile its per-point traversal without optimization. Preserve
# explicit Debug/RelWithDebInfo choices and multi-configuration generators.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_voxel_grid REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(rlai_voxel_layer SHARED src/ray_voxel_layer.cpp src/ray_obstacle_layer.cpp)
target_compile_features(rlai_voxel_layer PUBLIC cxx_std_17)
target_compile_options(rlai_voxel_layer PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(rlai_voxel_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rlai_voxel_layer nav2_costmap_2d nav2_voxel_grid pluginlib sensor_msgs rclcpp)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)
install(TARGETS rlai_voxel_layer LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_grid_ray test/test_grid_ray.cpp)
  target_compile_features(test_grid_ray PUBLIC cxx_std_17)
  target_include_directories(test_grid_ray PRIVATE include)
  add_test(NAME grid_ray_geometry COMMAND test_grid_ray)
  add_executable(test_native_ray test/test_native_ray.cpp)
  target_compile_features(test_native_ray PUBLIC cxx_std_17)
  target_include_directories(test_native_ray PRIVATE include)
  ament_target_dependencies(test_native_ray nav2_voxel_grid rclcpp)
  add_test(NAME native_ray_counterexample COMMAND test_native_ray)
  add_executable(test_obstacle_ray test/test_obstacle_ray.cpp)
  target_compile_features(test_obstacle_ray PUBLIC cxx_std_17)
  target_include_directories(test_obstacle_ray PRIVATE include)
  target_link_libraries(test_obstacle_ray rlai_voxel_layer)
  ament_target_dependencies(test_obstacle_ray nav2_costmap_2d sensor_msgs rclcpp)
  add_test(NAME obstacle_ray_regressions COMMAND test_obstacle_ray)
endif()
ament_package()
