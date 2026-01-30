"""抓取控制主节点：状态机管理完整抓取流程"""

from enum import Enum, auto
from typing import Optional, List
import time
import yaml
import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

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

        self.callback_group = ReentrantCallbackGroup()
        self.config = self._load_config()

        # 初始化子模块
        self.transformer = CoordinateTransformer(self, self.config)
        self.hand = HandController(self, self.config, self.callback_group)
        self.moveit = MoveItArmController(self, self.config)
        self.arm = ArmController(self, self.config, self.callback_group)

        # 状态机
        self.state = GraspState.IDLE
        self.state_executing = False
        self.target_position: Optional[List[float]] = None
        self.detection_start_time: Optional[float] = None

        # [修正 1] 增加对 rx/ry/rz 的解析逻辑
        positions = self.config.get('positions', {})
        
        # 1. 优先读取关节角度 (List[float])
        self.home_joints = positions.get('home_joints')
        self.handover_joints = positions.get('handover_joints')
        
        # 2. 解析 Pose (自动识别 rx/ry/rz 或 rpy)
        self.home_pose = self._parse_pose_from_config(positions.get('home_pose'))
        self.handover_pose = self._parse_pose_from_config(positions.get('handover_pose'))

        # [修正 2] 解析抓取姿态，统一转为四元数存储
        grasp_cfg = self.config.get('grasp', {})
        self.lift_height = grasp_cfg.get('lift_height', 0.20)
        
        orientation_cfg = grasp_cfg.get('orientation', {})
        # 这里会生成 np.array([x, y, z, w])
        self.grasp_orientation_q = self._parse_orientation_to_quaternion(orientation_cfg)

        # 安全配置
        safety = self.config.get('safety', {})
        self.detection_timeout = safety.get('detection_timeout', 10.0)

        # 订阅
        detection_cfg = self.config.get('detection', {})
        centers_topic = detection_cfg.get('centers_topic', '/centers')
        self.centers_sub = self.create_subscription(
            PoseArray, centers_topic, self._centers_callback, 10,
            callback_group=self.callback_group
        )
        self.latest_detection: Optional[Pose] = None

        self.trigger_srv = self.create_service(
            Trigger, '~/trigger', self._trigger_callback,
            callback_group=self.callback_group
        )

        self.timer = self.create_timer(
            0.1, self._state_machine_tick,
            callback_group=self.callback_group
        )

        self.get_logger().info('抓取控制节点已启动')

    def _load_config(self) -> dict:
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        if not config_file:
            try:
                pkg_dir = get_package_share_directory('grasp_control')
                config_file = os.path.join(pkg_dir, 'config', 'grasp_config.yaml')
            except Exception:
                config_file = '/home/rvl/ros2_ws/src/grasp_control/config/grasp_config.yaml'
        
        self.get_logger().info(f'加载配置: {config_file}')
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)

    def _parse_orientation_to_quaternion(self, cfg_dict):
        """[新增] 智能解析姿态：支持 rx/ry/rz 或 rpy"""
        if not cfg_dict:
            return self.transformer._rpy_to_quaternion(3.14, 0.0, 0.0)

        # 优先检查 rx, ry, rz (UR 旋转向量)
        if 'rx' in cfg_dict or 'ry' in cfg_dict or 'rz' in cfg_dict:
            rx = float(cfg_dict.get('rx', 0.0))
            ry = float(cfg_dict.get('ry', 0.0))
            rz = float(cfg_dict.get('rz', 0.0))
            return self.transformer._rotvec_to_quaternion(rx, ry, rz)
        
        # 否则使用 RPY (欧拉角)
        roll = float(cfg_dict.get('roll', 0.0))
        pitch = float(cfg_dict.get('pitch', 0.0))
        yaw = float(cfg_dict.get('yaw', 0.0))
        return self.transformer._rpy_to_quaternion(roll, pitch, yaw)

    def _parse_pose_from_config(self, cfg_dict):
        """[修正] 从配置解析 Pose，调用智能姿态解析"""
        if not cfg_dict:
            return None
        pose = Pose()
        pose.position.x = float(cfg_dict.get('x', 0.0))
        pose.position.y = float(cfg_dict.get('y', 0.0))
        pose.position.z = float(cfg_dict.get('z', 0.0))
        
        # 转换姿态
        q = self._parse_orientation_to_quaternion(cfg_dict)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def _centers_callback(self, msg: PoseArray):
        if len(msg.poses) > 0:
            self.latest_detection = msg.poses[0]
            if self.state == GraspState.DETECTING:
                self.get_logger().info(
                    f'检测到物体: 像素({self.latest_detection.position.x:.1f}, '
                    f'{self.latest_detection.position.y:.1f})'
                )

    def _trigger_callback(self, request, response):
        if self.state != GraspState.IDLE:
            response.success = False
            response.message = f'当前状态: {self.state.name}，无法触发'
            return response
        self._transition_to(GraspState.MOVE_HOME)
        response.success = True
        response.message = '抓取流程已启动'
        return response

    def _transition_to(self, new_state: GraspState):
        self.get_logger().info(f'状态转换: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_executing = False

    def _state_machine_tick(self):
        if self.state == GraspState.IDLE: pass
        elif self.state == GraspState.MOVE_HOME: self._handle_move_home()
        elif self.state == GraspState.DETECTING: self._handle_detecting()
        elif self.state == GraspState.CALCULATING: self._handle_calculating()
        elif self.state == GraspState.APPROACH: self._handle_approach()
        elif self.state == GraspState.GRASPING: self._handle_grasping()
        elif self.state == GraspState.LIFTING: self._handle_lifting()
        elif self.state == GraspState.HANDOVER: self._handle_handover()
        elif self.state == GraspState.RELEASING: self._handle_releasing()
        elif self.state == GraspState.COMPLETE:
            self.get_logger().info('流程完成')
            self._transition_to(GraspState.IDLE)
        elif self.state == GraspState.ERROR:
            self.get_logger().error('发生错误，重置状态')
            self._transition_to(GraspState.IDLE)

    def _handle_move_home(self):
        """[修正] 移动到 Home：优先关节控制，备用笛卡尔控制"""
        if self.state_executing: return
        self.state_executing = True
        self.get_logger().info('移动到 Home 位置...')
        
        self.latest_detection = None

        # 1. 优先：关节控制 (最稳健)
        if self.home_joints:
            self.get_logger().info('模式: 关节空间运动 (Home Joints)')
            self.hand.open_hand()
            if self.arm.move_to_joints_sync(self.home_joints):
                self.get_logger().info('到达 Home 点，等待相机稳定...')
                time.sleep(1.0)
                self.latest_detection = None
                self.detection_start_time = time.time()
                self._transition_to(GraspState.DETECTING)
            else:
                self._transition_to(GraspState.ERROR)
        
        # 2. 备用：Pose 控制 (如果您一定要用)
        elif self.home_pose:
            self.get_logger().warn('模式: 笛卡尔空间运动 (Home Pose)')
            self.hand.open_hand()
            if self.moveit.move_to_pose_sync(self.home_pose):
                time.sleep(1.0)
                self.latest_detection = None
                self.detection_start_time = time.time()
                self._transition_to(GraspState.DETECTING)
            else:
                self._transition_to(GraspState.ERROR)
        else:
            self.get_logger().error('Home 未配置 (需要 home_joints 或 home_pose)')
            self._transition_to(GraspState.ERROR)

    def _handle_detecting(self):
        if self.latest_detection is not None:
            self._transition_to(GraspState.CALCULATING)
            return
        if time.time() - self.detection_start_time > self.detection_timeout:
            self.get_logger().error('检测超时')
            self._transition_to(GraspState.ERROR)

    def _handle_calculating(self):
        if self.latest_detection is None:
            self._transition_to(GraspState.ERROR)
            return
        
        u = self.latest_detection.position.x
        v = self.latest_detection.position.y
        depth = self.latest_detection.position.z

        point_base = self.transformer.pixel_to_base(u, v, depth)

        if point_base is None:
            self._transition_to(GraspState.ERROR)
            return

        self.target_position = point_base.tolist()
        self.get_logger().info(f'目标基座坐标: {self.target_position}')
        self._transition_to(GraspState.APPROACH)

    def _handle_approach(self):
        """
        四步接近策略 (带关节限制)：
        0. 定义约束：锁死基座前方区域，防止乱转
        1. 高空平移 (PTP + Constraints): 飞到物体上方 30cm (保持当前姿态)
        2. 原地旋转 (Linear): 调整为抓取姿态
        3. 垂直下降A (Linear): 预抓取
        4. 垂直下降B (Linear): 抓取
        """
        if self.state_executing: return
        self.state_executing = True
        self.get_logger().info('>>> 开始接近流程 (四步 + 关节约束) <<<')

        # 准备位姿数据
        final_grasp_pose = self.transformer.compute_grasp_pose(
            self.target_position, self.grasp_orientation_q
        )
        current_tcp_pose = self.transformer.get_current_tcp_pose()
        if current_tcp_pose is None:
            self._transition_to(GraspState.ERROR)
            return

        # [关键] 定义关节约束 (单位: 弧度)
        # joint_constraints = {
        #     'shoulder_pan_joint': (1.1, 2.5), 
        #     'wrist_3_joint': (0, 5.0),
        #     'wrist_2_joint': (-2.0, -0.9),
        #     'wrist_1_joint': (-3.14, -1.5),
        # }

        # ---------------------------------------------------------
        # 步骤 1: 高空平移 (PTP)
        # ---------------------------------------------------------
        hover_pose = Pose()
        hover_pose.position.x = final_grasp_pose.position.x
        hover_pose.position.y = final_grasp_pose.position.y
        hover_pose.position.z = final_grasp_pose.position.z + 0.30
        hover_pose.orientation = current_tcp_pose.orientation # 保持姿态不变

        self.get_logger().info('步骤1: 高空平移 (带约束)...')
        
        # 传入 joint_limits 参数
        if not self.moveit.move_to_pose_sync(
            hover_pose, 
            velocity_scaling=0.4,
            # joint_limits=joint_constraints  # <--- 加上这一行
        ):
            self.get_logger().error('步骤1 失败: 规划无法满足关节约束 (可能目标点在死区)')
            self._transition_to(GraspState.ERROR)
            return

        # ---------------------------------------------------------
        # 步骤 2: 原地旋转 (Linear)
        # ---------------------------------------------------------
        rotate_pose = Pose()
        rotate_pose.position = hover_pose.position
        rotate_pose.orientation = final_grasp_pose.orientation

        self.get_logger().info('步骤2: 原地调整姿态...')
        if not self.moveit.move_cartesian_path([rotate_pose], step=0.01, speed_factor=0.1):
            self.get_logger().error('步骤2 失败')
            self._transition_to(GraspState.ERROR)
            return

        # ---------------------------------------------------------
        # 步骤 3: 垂直下降 (Linear)
        # ---------------------------------------------------------
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = final_grasp_pose.position.x
        pre_grasp_pose.position.y = final_grasp_pose.position.y
        pre_grasp_pose.position.z = final_grasp_pose.position.z + 0.10
        pre_grasp_pose.orientation = final_grasp_pose.orientation

        self.get_logger().info('步骤3: 垂直下降...')
        if not self.moveit.move_cartesian_path([pre_grasp_pose], step=0.01, speed_factor=0.2):
            self._transition_to(GraspState.ERROR)
            return

        # ---------------------------------------------------------
        # 步骤 4: 最终抓取
        # ---------------------------------------------------------
        self.get_logger().info('步骤4: 抵近抓取...')
        if self.moveit.move_cartesian_path([final_grasp_pose], step=0.005, speed_factor=0.05):
            self._transition_to(GraspState.GRASPING)
        else:
            self._transition_to(GraspState.ERROR)

    def _handle_grasping(self):
        if self.state_executing: return
        self.state_executing = True
        self.get_logger().info('抓取...')
        if self.hand.close_hand():
            time.sleep(0.5)
            self._transition_to(GraspState.LIFTING)
        else:
            self._transition_to(GraspState.ERROR)

    def _handle_lifting(self):
        if self.state_executing: return
        self.state_executing = True
        self.get_logger().info('执行: 慢速直线抬起...')
        
        current_pose = self.transformer.compute_grasp_pose(self.target_position, self.grasp_orientation_q)
        lift_pose = Pose()
        lift_pose.position.x = current_pose.position.x
        lift_pose.position.y = current_pose.position.y
        lift_pose.position.z = current_pose.position.z + self.lift_height
        lift_pose.orientation = current_pose.orientation

        # 同样使用 speed_factor=0.1
        if self.moveit.move_cartesian_path([lift_pose], step=0.01, speed_factor=0.1):
            self._transition_to(GraspState.HANDOVER)
        else:
            self.get_logger().error('抬起失败')
            self._transition_to(GraspState.ERROR)

    def _handle_handover(self):
        """[修正] 递交：优先关节控制"""
        if self.state_executing: return
        self.state_executing = True
        self.get_logger().info('前往递交点...')
        
        if self.handover_joints:
            self.get_logger().info('模式: 关节空间运动')
            if self.arm.move_to_joints_sync(self.handover_joints):
                self._transition_to(GraspState.RELEASING)
            else:
                self._transition_to(GraspState.ERROR)
        elif self.handover_pose:
            self.get_logger().info('模式: 笛卡尔空间运动')
            if self.moveit.move_to_pose_sync(self.handover_pose):
                self._transition_to(GraspState.RELEASING)
            else:
                self._transition_to(GraspState.ERROR)
        else:
            self._transition_to(GraspState.ERROR)

    def _handle_releasing(self):
        if self.state_executing: return
        self.state_executing = True
        self.get_logger().info('释放...')
        if self.hand.open_hand():
            time.sleep(0.5)
            self._transition_to(GraspState.COMPLETE)
        else:
            self._transition_to(GraspState.ERROR)


def main():
    rclpy.init()
    node = GraspControlNode()
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