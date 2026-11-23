"""
rlai_localization/launch/localization_amcl.launch.py

Standalone AMCL localisation launch — use this when you have a pre-built map
and want global localisation without starting a new SLAM session.

Starts:
  1. imu_filter_madgwick  — /imu/data_raw → /imu/data
  2. ekf_node             — fuses /wheel_odom + /imu/data → /odometry/filtered
                            + odom→base_footprint TF
  3. map_server           — serves the occupancy grid from map_yaml_file on /map
  4. amcl                 — particle-filter localisation against /map
                            publishes map→odom TF
  5. lifecycle_manager    — configure+activate map_server and amcl

Transform chain:
  map → odom → base_footprint

Usage:
  ros2 launch rlai_localization localization_amcl.launch.py \\
    map_yaml_file:=/path/to/my_map.yaml

  ros2 launch rlai_localization localization_amcl.launch.py \\
    map_yaml_file:=/home/user/maps/warehouse.yaml \\
    use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            'map_yaml_file',
            description='Absolute path to an occupancy map YAML file (required)',
        ),
    ]

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

    # Lifecycle node; managed by lifecycle_manager below.
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
    )

    # Lifecycle node; managed by lifecycle_manager below.
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
    )

    # Configure and activate map_server and amcl.
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
    )

    # Lifecycle transport is bounded; node names and stock bonds stay intact.
    lifecycle_guard = Node(
        package='rlai_localization', executable='bounded_lifecycle',
        name='localization_startup_guard', output='screen',
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': LaunchConfiguration('localization_fastdds_profile')},
        sigterm_timeout='3', sigkill_timeout='2',
        condition=None,
    )
    startup_actions = bounded_startup_actions(
        lifecycle_guard, LaunchConfiguration('localization_startup_timeout'),
        condition=None)

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
