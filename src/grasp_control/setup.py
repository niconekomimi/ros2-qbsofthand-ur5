from setuptools import setup
import os
from glob import glob

package_name = 'grasp_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RVL',
    maintainer_email='rvl@example.com',
    description='ROS2 视觉抓取控制包',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'grasp_node = grasp_control.grasp_node:main',
        ],
    },
)
