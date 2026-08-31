from setuptools import find_packages, setup

package_name = 'laser_scan_odometry'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Dubey',
    maintainer_email='aaryaganesh15@gmail.com',
    description='2D ICP scan-matching odometry for lidar-only platforms.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'icp_odom_node = laser_scan_odometry.icp_odom_node:main',
        ],
    },
)
