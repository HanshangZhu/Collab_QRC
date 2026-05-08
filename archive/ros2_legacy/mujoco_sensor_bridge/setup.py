import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'mujoco_sensor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hanshang Zhu',
    maintainer_email='hz@todo.todo',
    description='MuJoCo sensor bridge nodes for Go2W simulation',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mujoco_lidar_node = mujoco_sensor_bridge.mujoco_lidar_node:main',
            'mujoco_lidar_node_multiray = mujoco_sensor_bridge.mujoco_lidar_node_multiray:main',
            'mujoco_contact_node = mujoco_sensor_bridge.mujoco_contact_node:main',
            'mujoco_odom_bridge = mujoco_sensor_bridge.mujoco_odom_bridge:main',
        ],
    },
)
