"""
rlai_localization/launch/localization.launch.py

Localization stack for rbot.

Always started:
  - imu_filter_madgwick  (imu_tools)   — Madgwick complementary filter;
                                         /imu/data_raw → /imu/data (with orientation)
  - ekf_node  (robot_localization)     — fuses /wheel_odom + /imu/data,
                                         publishes odom→base_footprint TF
                                         and /odometry/filtered

Conditionally started when use_amcl:=true:
  - map_server  (nav2_map_server)      — serves /map from a YAML map file
  - amcl        (nav2_amcl)            — localises against /map, publishes
                                         map→odom TF

Transform chain after this launch:
  Without AMCL:  odom → base_footprint   (EKF only; no global reference)
  With AMCL:     map  → odom → base_footprint   (full chain)

Prerequisites:
  - /wheel_odom    (nav_msgs/Odometry)    — published by diff_drive_controller
  - /imu/data_raw  (sensor_msgs/Imu)      — published by Gazebo IMU plugin
  - /scan          (sensor_msgs/LaserScan) — required only when use_amcl:=true
  - A map YAML file at the path provided via map_yaml_file arg

Usage:
  ros2 launch rlai_localization localization.launch.py
  ros2 launch rlai_localization localization.launch.py use_amcl:=true \
    map_yaml_file:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rlai_localization.lifecycle_launch import bounded_startup_actions


def generate_launch_description():
    pkg = FindPackageShare('rlai_localization')

    imu_filter_yaml = PathJoinSubstitution([pkg, 'config', 'imu_filter.yaml'])
    ekf_yaml = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])
    amcl_yaml = PathJoinSubstitution([pkg, 'config', 'amcl.yaml'])
    service_profile = PathJoinSubstitution([pkg, 'config', 'localization_service_replies.xml'])

    # Launch arguments
    declared_args = [
        DeclareLaunchArgument(
            'localization_startup_timeout', default_value='25.0',
            description='Wall-clock startup deadline in seconds, finite range 5 to 120'),
        DeclareLaunchArgument(
            'localization_fastdds_profile',
            default_value=EnvironmentVariable(
                'FASTRTPS_DEFAULT_PROFILES_FILE', default_value=service_profile),
            description='Fast DDS XML for map_server and AMCL; preserve or merge a custom service profile',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock',
        ),
        DeclareLaunchArgument(
            'use_amcl',
            default_value='false',
            description=(
                'Start AMCL + map_server for global localisation. '
                'Requires map_yaml_file and should remain false during SLAM.'
            ),
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='',
            description='Absolute path to a map YAML file (required when use_amcl:=true)',
        ),
    ]

    # IMU filter node
    # Converts /imu/data_raw → /imu/data by running the Madgwick complementary
    # filter.  Provides orientation quaternion for downstream consumers (rviz2,
    # nav2).  EKF fuses only gyro-z and accel-x/y from /imu/data — not the
    # orientation quaternion — to avoid discontinuities on a flat-floor robot.
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[
            imu_filter_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data',     '/imu/data'),
        ],
    )

    # EKF node
    # Fuses wheel odometry + filtered IMU.  Sole publisher of odom→base_footprint TF.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            ekf_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # map_yaml_file must be non-empty when use_amcl is true.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        remappings=[('map_server/change_state', '_localization_control/map_server/change_state'),
                    ('map_server/get_state', '_localization_control/map_server/get_state')],
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': LaunchConfiguration('localization_fastdds_profile')},
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map_yaml_file'),
        }],
        condition=IfCondition(LaunchConfiguration('use_amcl')),
    )

    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        remappings=[('amcl/change_state', '_localization_control/amcl/change_state'),
                    ('amcl/get_state', '_localization_control/amcl/get_state')],
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': LaunchConfiguration('localization_fastdds_profile')},
        name='amcl',
        output='screen',
        parameters=[
            amcl_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('use_amcl')),
    )

    # Lifecycle manager for map_server and amcl
    # Transitions both nodes through configure→activate automatically.
    # Only started when use_amcl is true.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=IfCondition(LaunchConfiguration('use_amcl')),
    )

    # Lifecycle transport is bounded; node names and stock bonds stay intact.
    lifecycle_guard = Node(
        package='rlai_localization', executable='bounded_lifecycle',
        name='localization_startup_guard', output='screen',
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': LaunchConfiguration('localization_fastdds_profile')},
        sigterm_timeout='3', sigkill_timeout='2',
        condition=IfCondition(LaunchConfiguration('use_amcl')),
    )
    startup_actions = bounded_startup_actions(
        lifecycle_guard, LaunchConfiguration('localization_startup_timeout'),
        condition=IfCondition(LaunchConfiguration('use_amcl')))

    return LaunchDescription(
        declared_args + startup_actions + [
            lifecycle_guard,
            imu_filter_node,
            ekf_node,
            map_server,
            amcl_node,
            lifecycle_manager,
        ]
    )
