"""Laptop 2 (this machine) -- the control station.

Consumes /scan from the lidar laptop over WiFi and runs the whole mapping
stack locally: ICP odometry, slam_toolbox, and RViz2.  Keeping SLAM on this side
means only the raw scan crosses the network, and the map never does.

    ros2 launch lidar_slam_bringup control_station.launch.py

Frame chain when this is running:
    map -> odom          slam_toolbox   (corrects the drift below)
    odom -> base_link    icp_odom_node  (dead reckoning from scan matching)
    base_link -> laser_frame            (static, published by laptop 1)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (
    EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = FindPackageShare('lidar_slam_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Set false to run the mapping stack headless.'),
        DeclareLaunchArgument(
            'odometry', default_value='icp',
            choices=['icp', 'static', 'external'],
            description="Source of odom -> base_link. 'icp' runs the scan "
                        "matcher (default). 'static' pins odom to base_link and "
                        "leans entirely on slam_toolbox's matcher -- only usable "
                        "at walking pace. 'external' means the lidar laptop "
                        "already publishes odometry."),
        DeclareLaunchArgument(
            'publish_laser_tf', default_value='false',
            description='Publish base_link -> laser_frame from here. Normally '
                        'the lidar laptop owns this, since it owns the physical '
                        'mount -- set true only if that machine publishes no TF.'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.18'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [pkg, 'config', 'slam_toolbox_async.yaml']),
            description='slam_toolbox parameter file.'),
    ]

    icp_odom = Node(
        package='laser_scan_odometry',
        executable='icp_odom_node',
        name='icp_odom_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('odometry'), 'icp')),
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'icp_odometry.yaml']),
            {'use_sim_time': use_sim_time},
        ],
    )

    # Fallback: no odometry estimate at all, odom and base_link coincide.
    static_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom',
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('odometry'), 'static')),
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'odom', '--child-frame-id', 'base_link',
        ],
    )

    # Fallback for a lidar laptop that publishes /scan but no transforms.
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_laser_tf')),
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--roll', '0.0', '--pitch', '0.0',
            '--yaw', LaunchConfiguration('laser_yaw'),
            '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
        ],
    )

    # slam_toolbox is a managed (lifecycle) node: constructing it is not enough,
    # it sits in 'unconfigured' until told otherwise. Until it is configured it
    # declares no parameters, subscribes to nothing, and publishes no map -- and
    # it does all that silently, so it looks like a healthy node doing nothing.
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': use_sim_time, 'use_lifecycle_manager': False},
        ],
    )

    # unconfigured -> inactive
    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    # inactive -> active, once configuration actually succeeded
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='slam_toolbox configured; activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'slam_view.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(args + [
        LogInfo(msg=['Control station up. Odometry source: ',
                     LaunchConfiguration('odometry'), '.']),
        icp_odom,
        static_odom,
        base_to_laser,
        slam,
        slam_configure,
        slam_activate,
        rviz,
    ])
