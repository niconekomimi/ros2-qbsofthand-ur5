"""抓取控制启动文件"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('grasp_control')
    config_file = os.path.join(pkg_dir, 'config', 'grasp_config.yaml')

    # 手眼标定 TF 发布（可选，如果已经单独启动则注释掉）
    # handeye_publisher = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('easy_handeye2'),
    #         '/launch/publish.launch.py'
    #     ]),
    #     launch_arguments={'name': 'ur5_oak_eyehand'}.items()
    # )

    # 抓取控制节点
    grasp_node = Node(
        package='grasp_control',
        executable='grasp_node',
        name='grasp_control_node',
        output='screen',
        parameters=[{
            'config_file': config_file
        }]
    )

    return LaunchDescription([
        # handeye_publisher,  # 取消注释以自动启动手眼标定发布
        grasp_node,
    ])
