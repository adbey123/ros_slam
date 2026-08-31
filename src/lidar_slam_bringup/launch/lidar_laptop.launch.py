"""Laptop 1 -- the machine the YDLIDAR is plugged into.

Brings up the lidar driver and the static transforms that describe where the
lidar sits on the platform.  Nothing else: no SLAM, no RViz.  /scan and the
base_link -> laser_frame transform go out over WiFi and the control station
picks them up.

    ros2 launch lidar_slam_bringup lidar_laptop.launch.py lidar_model:=x4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('lidar_slam_bringup')

    args = [
        DeclareLaunchArgument(
            'lidar_model', default_value='x4',
            choices=['x2', 'x4', 'g2'],
            description='Which YDLIDAR profile to load from config/.'),
        DeclareLaunchArgument(
            'port', default_value='/dev/ydlidar',
            description='Serial device. /dev/ydlidar comes from the udev rule; '
                        'use /dev/ttyUSB0 if you skipped installing it.'),
        DeclareLaunchArgument(
            'laser_frame', default_value='laser_frame',
            description='Frame the driver stamps on each scan.'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Platform body frame.'),
        # Where the lidar is mounted relative to base_link, in metres/radians.
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.18'),
        DeclareLaunchArgument(
            'laser_yaw', default_value='0.0',
            description='Set this if the lidar 0-degree mark does not point '
                        'forward. A wrong value makes the map shear as you turn.'),
    ]

    lidar_config = PathJoinSubstitution([
        pkg, 'config',
        ['ydlidar_', LaunchConfiguration('lidar_model'), '.yaml'],
    ])

    driver = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            lidar_config,
            {
                'port': LaunchConfiguration('port'),
                'frame_id': LaunchConfiguration('laser_frame'),
            },
        ],
    )

    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--yaw', LaunchConfiguration('laser_yaw'),
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('laser_frame'),
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg=['Lidar laptop up. Publishing /scan and ',
                     LaunchConfiguration('base_frame'), ' -> ',
                     LaunchConfiguration('laser_frame'), '.']),
        driver,
        base_to_laser,
    ])
