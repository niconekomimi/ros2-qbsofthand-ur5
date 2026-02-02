"""一键启动所有抓取相关节点"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch 参数
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.1.211',
        description='UR5 机械臂 IP 地址'
    )
    reverse_ip_arg = DeclareLaunchArgument(
        'reverse_ip', default_value='192.168.1.10',
        description='本机 IP 地址'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    reverse_ip = LaunchConfiguration('reverse_ip')

    # 获取包路径
    grasp_pkg = get_package_share_directory('grasp_control')
    config_file = os.path.join(grasp_pkg, 'config', 'grasp_config.yaml')
    yolo_params_file = PathJoinSubstitution([
        FindPackageShare('yolo_center_detector'),
        'config',
        'extended_disp.yaml'
    ])

    # 1. 相机
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('depthai_ros_driver'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': yolo_params_file
        }.items()
    )

    # 2. UR5 机械臂
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ur_robot_driver'),
            '/launch/ur_control.launch.py'
        ]),
        launch_arguments={
            'ur_type': 'ur5',
            'robot_ip': robot_ip,
            'reverse_ip': reverse_ip,
            'launch_rviz': 'false',
            'tf_prefix': '',  # [关键修正] 显式设置为空，防止污染关节名称
        }.items()
    )

    # 3. 手眼标定 TF 发布（延迟 3 秒启动，等待 TF 树建立）
    handeye_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('easy_handeye2'),
                    '/launch/publish.launch.py'
                ]),
                launch_arguments={'name': 'ur5_oak_eyehand'}.items()
            )
        ]
    )

    # 4. 灵巧手
    softhand_node = Node(
        package='qbsofthand_control',
        executable='qbsofthand_control_node',
        name='qbsofthand_control_node',
        output='screen',
    )

    # 5. YOLO 检测（延迟 5 秒启动，等待相机就绪）
    yolo_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('yolo_center_detector'),
                    '/launch/yolo_center.launch.py'
                ])
            )
        ]
    )

    # 6. MoveIt2（延迟 5 秒启动，等待机械臂就绪）
    moveit_launch = TimerAction(
        period=5.0, # 如果机器启动较慢，可以适当增加到 8.0
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('ur_moveit_config'),
                    '/launch/ur_moveit.launch.py'
                ]),
                launch_arguments={
                    'ur_type': 'ur5',            # 确保型号正确
                    'robot_ip': robot_ip,        # [关键] 必须传入 IP 以加载正确的描述/标定
                    'launch_rviz': 'false',      # [建议] 如果不需要看 MoveIt 界面，设为 false 防止卡顿
                    'use_fake_hardware': 'false', # 显式声明使用真实硬件
                    'tf_prefix': '',  # [关键修正] 显式设置为空，防止污染关节名称
                }.items()
            )
        ]
    )

    # 7. 抓取控制节点（延迟 10 秒启动，等待所有依赖就绪）
    grasp_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='grasp_control',
                executable='grasp_node',
                name='grasp_control_node',
                output='screen',
                parameters=[{'config_file': config_file}]
            )
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        reverse_ip_arg,
        camera_launch,
        ur_launch,
        handeye_launch,
        softhand_node,
        yolo_launch,
        moveit_launch,
        grasp_node,
    ])
