"""Launch the Nav2 planning, control, recovery, and behavior-tree stack.

The localization stack owns map_server, AMCL, and map->odom.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_navigation(context):

    guarded = LaunchConfiguration('use_motion_supervisor').perform(context).lower() in ('true','1','yes','on')
    velocity_remaps = [('cmd_vel','nav2_adapter/cmd_vel_input')] if guarded else []
    pkg = FindPackageShare('rlai_navigation')
    nav2_params_yaml = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])

    parameter_files = [nav2_params_yaml]
    if LaunchConfiguration('use_3d_obstacles').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        parameter_files.append(PathJoinSubstitution([pkg, 'config', 'nav2_lidar3d_overlay.yaml']))

    occlusion_enabled = LaunchConfiguration('use_scan_occlusion').perform(context).lower() in ('true','1','yes','on')
    if occlusion_enabled:
        parameter_files.append(PathJoinSubstitution([pkg, 'config', 'scan_occlusion_overlay.yaml']))

    # Navigation planning and control nodes

    # SMAC Hybrid-A* global planner.
    # Owns the global_costmap; publishes /plan on goal dispatch.
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=parameter_files + [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # SimpleSmoother applied to the global path before the controller receives it.
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=parameter_files + [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # MPPI local controller.
    # Owns the local_costmap; publishes /cmd_vel, /local_plan, /trajectories.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        remappings=velocity_remaps,
        output='screen',
        parameters=parameter_files + [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Multi-goal waypoint sequencer — FollowWaypoints action server.
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=parameter_files + [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Transitions all navigation nodes through configure→activate.
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=parameter_files + [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Recovery behavior and behavior-tree nodes

    # Recovery behavior plugins: spin, backup, drive_on_heading, wait.
    # bt_navigator calls these action servers when a navigation step fails.
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        remappings=velocity_remaps,
        output='screen',
        parameters=parameter_files + [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Behavior tree orchestrator.
    # Exposes NavigateToPose + FollowWaypoints action servers.
    # Resolves BT XML files from the rlai_navigation package share directory.
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        remappings=[('navigate_to_pose','nav2_internal/navigate_to_pose'),
                    ('navigate_through_poses','nav2_internal/navigate_through_poses')] if guarded else [],
        output='screen',
        parameters=parameter_files + [
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Resolve the default BT XML to the full installed share path.
                # In Nav2 Jazzy the param was renamed from default_bt_xml_filename.
                'default_nav_to_pose_bt_xml': PathJoinSubstitution([
                    pkg, 'behavior_trees', 'navigate_to_pose.xml'
                ]),
            },
        ],
    )

    # Transitions behavior_server → bt_navigator to ACTIVE.
    # Started last so action servers are already ACTIVE
    # before bt_navigator attempts to connect to them.
    lifecycle_manager_behavior = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_behavior',
        output='screen',
        parameters=parameter_files + [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    adapters = [Node(package='rlai_motion_supervisor',executable='nav_adapter',
        name='nav2_lease_adapter',output='screen',
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time'),'require_scan_occlusion':occlusion_enabled,'trace_occlusion_receiver':LaunchConfiguration('trace_occlusion_receiver')}])] if guarded else []

    occlusion_nodes = [Node(package='rlai_navigation', executable='scan_occlusion',
        name='scan_occlusion', output='screen',
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}])] if occlusion_enabled else []

    return adapters + occlusion_nodes + [
        planner_server,
        smoother_server,
        controller_server,
        waypoint_follower,
        lifecycle_manager_navigation,
        behavior_server,
        bt_navigator,
        lifecycle_manager_behavior,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('trace_occlusion_receiver',default_value='false',
            description='Publish receiver timing diagnostics; requires scan occlusion'),
        DeclareLaunchArgument('use_scan_occlusion',default_value='false',
            description='Reserve up to 1 m of unseen depth behind finite scan returns in global planning'),
        DeclareLaunchArgument('use_motion_supervisor',default_value='false',
            description='Route goals and velocities through the health-required lease-aware proxy'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument(
            'use_3d_obstacles', default_value='false',
            description='Add filtered 3D LiDAR voxel obstacle layers; requires the 3D sensor and filter',
        ),
        OpaqueFunction(function=_launch_navigation),
    ])
