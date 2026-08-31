"""Everything on one laptop -- for bench testing before you split the setup.

Plug the YDLIDAR into this machine and run:

    ros2 launch lidar_slam_bringup single_machine.launch.py lidar_model:=x4

If mapping works here but not across the two laptops, the problem is the
network, not the SLAM configuration. That is worth knowing before you debug DDS.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('lidar_slam_bringup')

    def include(name, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', name])),
            launch_arguments=kwargs.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_model', default_value='x4',
                              choices=['x2', 'x4', 'g2']),
        DeclareLaunchArgument('port', default_value='/dev/ydlidar'),
        DeclareLaunchArgument('odometry', default_value='icp',
                              choices=['icp', 'static', 'external']),
        DeclareLaunchArgument('rviz', default_value='true'),

        include('lidar_laptop.launch.py',
                lidar_model=LaunchConfiguration('lidar_model'),
                port=LaunchConfiguration('port')),
        include('control_station.launch.py',
                odometry=LaunchConfiguration('odometry'),
                rviz=LaunchConfiguration('rviz')),
    ])
