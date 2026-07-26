from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Create the Nav2 bringup launch description.

    The generated launch description declares the public arguments for the
    navigation entry point, resolves package-share paths, and starts the core
    Nav2 stack including the lifecycle manager.

    Launch arguments:
        use_sim_time (str): Use the /clock published by Gazebo.
            Defaults to ``"true"``.
        autostart (str): Automatically configure+activate the lifecycle nodes.
            Defaults to ``"true"``.

    Started processes:
        controller_server: LOCAL planning + local costmap.
        smoother_server: Path smoother (post-processes the global plan).
        planner_server: GLOBAL planning + global costmap.
        behavior_server: Recovery behaviors.
        bt_navigator: Behavior Tree orchestrator.
        waypoint_follower: Waypoint / multi-goal follower.
        velocity_smoother: Final anti-jerk stage.
        lifecycle_manager_navigation: Brings all of the above up in order.

    Returns:
        LaunchDescription: Ordered launch actions for the complete Nav2 stack.
    """

    ###########################################################################################################

    # <!-- Shared packages -->
    waver_navigation_pkg = FindPackageShare('waver_navigation')
    ###########################################################################################################

    # <!-- Config arguments -->
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    ###########################################################################################################

    # <!-- Paths -->
    costmaps_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'param', 'nav2', 'costmaps.yaml']
    )

    controller_params_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'param', 'nav2', 'controller_server.yaml']
    )

    planner_params_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'param', 'nav2', 'planner_server.yaml']
    )

    smoother_params_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'param', 'nav2', 'smoother_server.yaml']
    )

    behavior_params_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'param', 'nav2', 'behavior_server.yaml']
    )

    bt_params_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'param', 'nav2', 'bt_navigator.yaml']
    )

    waypoint_params_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'param', 'nav2', 'waypoint_follower.yaml']
    )

    velsmoother_params_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'param', 'nav2', 'velocity_smoother.yaml']
    )

    nav_to_pose_bt_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml']
    )

    nav_through_poses_bt_path = PathJoinSubstitution(
        [waver_navigation_pkg, 'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml']
    )
    ###########################################################################################################

    # <!-- Declare arguments -->
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the /clock published by Gazebo.'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure+activate the lifecycle nodes.'
    )
    ###########################################################################################################

    # <!-- controller_server -->
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            controller_params_path,
            costmaps_path,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
            ('odom', '/odom'),
            ('scan', '/scan'),
            ('local_plan', '/local_plan'),
        ],
    )

    # <!-- smoother_server -->
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            smoother_params_path,
            {'use_sim_time': use_sim_time}
        ],
    )

    # <!-- planner_server -->
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            planner_params_path,
            costmaps_path,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('plan', '/plan'),
            ('scan', '/scan'),
        ],
    )

    # <!-- behavior_server -->
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            behavior_params_path,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
            ('odom', '/odom'),
        ],
    )

    # <!-- bt_navigator -->
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_params_path,
            {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': nav_to_pose_bt_path,
                'default_nav_through_poses_bt_xml': nav_through_poses_bt_path,
            }
        ],
        remappings=[('odom', '/odom')],
    )

    # <!-- waypoint_follower -->
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            waypoint_params_path,
            {'use_sim_time': use_sim_time}
        ],
    )

    # <!-- velocity_smoother -->
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            velsmoother_params_path,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel'),
            ('odom', '/odom'),
        ],
    )

    # <!-- lifecycle_manager_navigation -->
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ],
    )
    ###########################################################################################################

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_autostart_cmd,
            controller_server_node,
            smoother_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            velocity_smoother_node,
            lifecycle_manager_navigation_node
        ]
    )
