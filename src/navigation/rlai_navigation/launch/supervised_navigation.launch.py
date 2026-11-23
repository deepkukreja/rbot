"""Guarded public NavigateToPose; requires the guarded simulation/health source first."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('trace_occlusion_receiver',default_value='false'),
        DeclareLaunchArgument('use_scan_occlusion',default_value='false'),
        DeclareLaunchArgument('use_3d_obstacles',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rlai_navigation'),'launch','navigation.launch.py'])),launch_arguments={
                'use_motion_supervisor':'true','trace_occlusion_receiver':LaunchConfiguration('trace_occlusion_receiver'),'use_sim_time':LaunchConfiguration('use_sim_time'),
                'use_3d_obstacles':LaunchConfiguration('use_3d_obstacles'),
                'use_scan_occlusion':LaunchConfiguration('use_scan_occlusion')}.items())])
