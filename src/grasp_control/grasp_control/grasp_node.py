"""抓取控制主节点：状态机管理完整抓取流程"""

from enum import Enum, auto
from typing import Optional, List
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import os

from .coordinate_transformer import CoordinateTransformer
from .arm_controller import ArmController
from .hand_controller import HandController
from .moveit_arm_controller import MoveItArmController


class GraspState(Enum):
    """抓取状态枚举"""
    IDLE = auto()        # 空闲
    MOVE_HOME = auto()   # 移动到拍照点
    DETECTING = auto()   # 检测物体
    CALCULATING = auto() # 计算坐标
    APPROACH = auto()    # 接近物体
    GRASPING = auto()    # 抓取
    LIFTING = auto()     # 抬起
    HANDOVER = auto()    # 递交
    RELEASING = auto()   # 释放
    COMPLETE = auto()    # 完成
    ERROR = auto()       # 错误


class GraspControlNode(Node):
    """抓取控制主节点"""

    def __init__(self):
        super().__init__('grasp_control_node')

        # 使用可重入回调组，允许回调嵌套
        self.callback_group = ReentrantCallbackGroup()

        # 加载配置
        self.config = self._load_config()

        # 初始化子模块（传递回调组）
        self.transformer = CoordinateTransformer(self, self.config)
        self.arm = ArmController(self, self.config, self.callback_group)
        self.hand = HandController(self, self.config, self.callback_group)
        self.moveit = MoveItArmController(self, self.config, self.callback_group)

        # 状态机
        self.state = GraspState.IDLE
        self.state_executing = False  # 标志：当前状态是否正在执行
        self.target_position: Optional[List[float]] = None
        self.detection_start_time: Optional[float] = None

        # 配置参数
        positions = self.config.get('positions', {})
        self.home_joints = positions.get('home', [-1.57, -1.57, 1.57, -1.57, -1.57, 0.0])
        self.handover_joints = positions.get('handover', [-0.78, -1.05, 1.57, -2.09, -1.57, 0.0])

        grasp_cfg = self.config.get('grasp', {})
        self.approach_height = grasp_cfg.get('approach_height', 0.03)
        self.lift_height = grasp_cfg.get('lift_height', 0.10)
        orientation = grasp_cfg.get('orientation', {})
        self.grasp_rpy = (
            orientation.get('roll', 3.14159),
            orientation.get('pitch', 0.0),
            orientation.get('yaw', 0.0)
        )

        safety = self.config.get('safety', {})
        self.detection_timeout = safety.get('detection_timeout', 10.0)

        # 订阅 YOLO 检测结果
        detection_cfg = self.config.get('detection', {})
        centers_topic = detection_cfg.get('centers_topic', '/centers')
        self.target_class = detection_cfg.get('target_class', 'bottle')

        self.centers_sub = self.create_subscription(
            PoseArray, centers_topic, self._centers_callback, 10,
            callback_group=self.callback_group
        )
        self.latest_detection: Optional[Pose] = None

        # 触发抓取服务
        self.trigger_srv = self.create_service(
            Trigger, '~/trigger', self._trigger_callback,
            callback_group=self.callback_group
        )

        # 状态机定时器
        self.timer = self.create_timer(
            0.1, self._state_machine_tick,
            callback_group=self.callback_group
        )

        self.get_logger().info('抓取控制节点已启动')
        self.get_logger().info(f'触发服务: {self.get_name()}/trigger')

    def _load_config(self) -> dict:
        """加载配置文件"""
        # 尝试从参数获取配置路径
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        if not config_file:
            # 使用默认路径
            try:
                pkg_dir = get_package_share_directory('grasp_control')
                config_file = os.path.join(pkg_dir, 'config', 'grasp_config.yaml')
            except Exception:
                # 开发时使用源码路径
                config_file = '/home/rvl/ros2_ws/src/grasp_control/config/grasp_config.yaml'

        self.get_logger().info(f'加载配置: {config_file}')

        try:
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'配置加载失败: {e}')
            return {}

    def _centers_callback(self, msg: PoseArray):
        """处理 YOLO 检测结果"""
        # 调试：记录回调被触发（改为 INFO 级别以便查看）
        self.get_logger().info(
            f'[DEBUG] 收到检测消息: poses数量={len(msg.poses)}, 当前状态={self.state.name}'
        )

        if len(msg.poses) > 0:
            # 无论在什么状态，都保存最新的检测结果
            # 这样在进入 DETECTING 状态时可以立即使用
            self.latest_detection = msg.poses[0]

            if self.state == GraspState.DETECTING:
                self.get_logger().info(
                    f'检测到物体: 像素({self.latest_detection.position.x:.1f}, '
                    f'{self.latest_detection.position.y:.1f}), '
                    f'深度={self.latest_detection.position.z:.3f}m'
                )
            else:
                self.get_logger().info(
                    f'[DEBUG] 收到检测但状态不是DETECTING: {self.state.name}，已保存检测结果'
                )

    def _trigger_callback(self, request, response):
        """触发抓取服务回调"""
        self.get_logger().info(f'[DEBUG] _trigger_callback 被调用，当前状态: {self.state.name}')

        if self.state != GraspState.IDLE:
            response.success = False
            response.message = f'当前状态: {self.state.name}，无法触发'
            return response

        self.get_logger().info('收到抓取触发请求')
        self._transition_to(GraspState.MOVE_HOME)
        response.success = True
        response.message = '抓取流程已启动'
        return response

    def _transition_to(self, new_state: GraspState):
        """状态转换"""
        self.get_logger().info(f'状态转换: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_executing = False  # 重置执行标志

    def _state_machine_tick(self):
        """状态机主循环"""
        # 调试：记录定时器触发
        self.get_logger().info(f'[DEBUG] tick: state={self.state.name}, executing={self.state_executing}')

        if self.state == GraspState.IDLE:
            pass  # 等待触发

        elif self.state == GraspState.MOVE_HOME:
            self._handle_move_home()

        elif self.state == GraspState.DETECTING:
            self._handle_detecting()

        elif self.state == GraspState.CALCULATING:
            self._handle_calculating()

        elif self.state == GraspState.APPROACH:
            self._handle_approach()

        elif self.state == GraspState.GRASPING:
            self._handle_grasping()

        elif self.state == GraspState.LIFTING:
            self._handle_lifting()

        elif self.state == GraspState.HANDOVER:
            self._handle_handover()

        elif self.state == GraspState.RELEASING:
            self._handle_releasing()

        elif self.state == GraspState.COMPLETE:
            self.get_logger().info('抓取流程完成！')
            self._transition_to(GraspState.IDLE)

        elif self.state == GraspState.ERROR:
            self.get_logger().error('抓取流程出错，返回空闲状态')
            self._transition_to(GraspState.IDLE)

    def _handle_move_home(self):
        """处理移动到 Home 位置"""
        if self.state_executing:
            return  # 操作正在执行，跳过

        self.state_executing = True
        self.get_logger().info('移动到 Home 位置（拍照点）...')

        # 清空之前的检测结果
        self.latest_detection = None

        # 先张开灵巧手
        self.hand.open_hand()
        self.get_logger().info('[DEBUG] 灵巧手张开完成，等待 0.5 秒...')
        time.sleep(0.5)

        # 移动到 Home
        self.get_logger().info('[DEBUG] 开始调用 move_to_joints_sync...')
        success = self.arm.move_to_joints_sync(self.home_joints)
        self.get_logger().info(f'[DEBUG] move_to_joints_sync 返回: {success}')

        if success:
            self.detection_start_time = time.time()
            self._transition_to(GraspState.DETECTING)
        else:
            self._transition_to(GraspState.ERROR)

    def _handle_detecting(self):
        """处理物体检测"""
        # 调试日志
        self.get_logger().info(
            f'[DEBUG] _handle_detecting: latest_detection={self.latest_detection is not None}'
        )

        if self.latest_detection is not None:
            self.get_logger().info(
                f'检测到物体，准备计算坐标: 像素({self.latest_detection.position.x:.1f}, '
                f'{self.latest_detection.position.y:.1f}), 深度={self.latest_detection.position.z:.3f}m'
            )
            self._transition_to(GraspState.CALCULATING)
            return

        # 检查超时
        elapsed = time.time() - self.detection_start_time
        if elapsed > self.detection_timeout:
            self.get_logger().error('物体检测超时')
            self._transition_to(GraspState.ERROR)

    def _handle_calculating(self):
        """处理坐标计算"""
        if self.latest_detection is None:
            self._transition_to(GraspState.ERROR)
            return

        # 像素坐标 + 深度 → 基座坐标
        u = self.latest_detection.position.x
        v = self.latest_detection.position.y
        depth = self.latest_detection.position.z

        point_base = self.transformer.pixel_to_base(u, v, depth)

        if point_base is None:
            self.get_logger().error('坐标转换失败')
            self._transition_to(GraspState.ERROR)
            return

        self.target_position = point_base.tolist()
        self.get_logger().info(
            f'目标位置（基座坐标）: '
            f'x={self.target_position[0]:.3f}, '
            f'y={self.target_position[1]:.3f}, '
            f'z={self.target_position[2]:.3f}'
        )

        self._transition_to(GraspState.APPROACH)

    def _handle_approach(self):
        """处理接近物体"""
        if self.state_executing:
            return  # 操作正在执行，跳过

        if self.target_position is None:
            self._transition_to(GraspState.ERROR)
            return

        self.state_executing = True

        # 计算接近位姿（物体上方）
        approach_pose = self.transformer.compute_grasp_pose(
            self.target_position,
            self.approach_height,
            self.grasp_rpy
        )

        self.get_logger().info(
            f'接近位置: x={approach_pose.position.x:.3f}, '
            f'y={approach_pose.position.y:.3f}, '
            f'z={approach_pose.position.z:.3f}'
        )

        # 使用 MoveIt2 进行笛卡尔运动
        success = self.moveit.move_to_pose_sync(approach_pose)

        if success:
            self._transition_to(GraspState.GRASPING)
        else:
            self.get_logger().error('接近运动失败')
            self._transition_to(GraspState.ERROR)

    def _handle_grasping(self):
        """处理抓取"""
        if self.state_executing:
            return  # 操作正在执行，跳过

        self.state_executing = True
        self.get_logger().info('闭合灵巧手抓取...')
        success = self.hand.close_hand()

        if success:
            time.sleep(0.5)  # 等待抓稳
            self._transition_to(GraspState.LIFTING)
        else:
            self._transition_to(GraspState.ERROR)

    def _handle_lifting(self):
        """处理抬起物体"""
        if self.state_executing:
            return  # 操作正在执行，跳过

        if self.target_position is None:
            self._transition_to(GraspState.ERROR)
            return

        self.state_executing = True
        self.get_logger().info('抬起物体...')

        # 计算抬起位姿（当前位置上方）
        lift_pose = self.transformer.compute_grasp_pose(
            self.target_position,
            self.lift_height,
            self.grasp_rpy
        )

        # 使用 MoveIt2 抬起
        success = self.moveit.move_to_pose_sync(lift_pose)

        if success:
            self._transition_to(GraspState.HANDOVER)
        else:
            self.get_logger().error('抬起运动失败')
            self._transition_to(GraspState.ERROR)

    def _handle_handover(self):
        """处理递交"""
        if self.state_executing:
            return  # 操作正在执行，跳过

        self.state_executing = True
        self.get_logger().info('移动到递交位置...')
        success = self.arm.move_to_joints_sync(self.handover_joints)

        if success:
            self._transition_to(GraspState.RELEASING)
        else:
            self._transition_to(GraspState.ERROR)

    def _handle_releasing(self):
        """处理释放"""
        if self.state_executing:
            return  # 操作正在执行，跳过

        self.state_executing = True
        self.get_logger().info('张开灵巧手释放物体...')
        success = self.hand.open_hand()

        if success:
            time.sleep(0.5)
            self._transition_to(GraspState.COMPLETE)
        else:
            self._transition_to(GraspState.ERROR)


def main():
    rclpy.init()
    node = GraspControlNode()

    # 使用多线程执行器
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
