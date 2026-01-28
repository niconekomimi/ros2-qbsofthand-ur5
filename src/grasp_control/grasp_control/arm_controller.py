"""机械臂控制模块：UR5 机械臂运动控制"""

import time
from typing import List, Optional
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


class ArmController:
    """UR5 机械臂控制器"""

    # UR5 关节名称
    JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    def __init__(self, node: Node, config: dict, callback_group=None):
        self.node = node
        self.config = config
        self.logger = node.get_logger()
        self.callback_group = callback_group

        # 配置参数
        arm_cfg = config.get('arm', {})
        action_name = arm_cfg.get(
            'action',
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.default_time = arm_cfg.get('trajectory_time', 4.0)
        self.approach_time = arm_cfg.get('approach_time', 3.0)

        # Action 客户端（使用回调组）
        self.action_client = ActionClient(
            node,
            FollowJointTrajectory,
            action_name,
            callback_group=callback_group
        )

        # 当前关节状态
        self.current_joints: Optional[List[float]] = None
        self.joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=callback_group
        )

        self.logger.info(f'机械臂控制器初始化完成，Action: {action_name}')

    def _joint_state_callback(self, msg: JointState):
        """更新当前关节状态"""
        # 按照 JOINT_NAMES 顺序提取关节角度
        try:
            positions = []
            for name in self.JOINT_NAMES:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_joints = positions
        except (ValueError, IndexError):
            pass  # 关节名称不匹配，忽略

    def get_current_joints(self) -> Optional[List[float]]:
        """获取当前关节角度"""
        return self.current_joints

    async def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """等待 Action 服务器就绪"""
        self.logger.info('等待机械臂控制器...')
        ready = self.action_client.wait_for_server(timeout_sec=timeout_sec)
        if ready:
            self.logger.info('机械臂控制器已就绪')
        else:
            self.logger.error('机械臂控制器连接超时')
        return ready

    async def move_to_joints(
        self,
        joint_positions: List[float],
        duration_sec: float = None
    ) -> bool:
        """
        移动到指定关节角度

        Args:
            joint_positions: 6 个关节角度 [rad]
            duration_sec: 运动时间（秒），None 使用默认值

        Returns:
            是否成功
        """
        if len(joint_positions) != 6:
            self.logger.error(f'关节数量错误: {len(joint_positions)}, 需要 6 个')
            return False

        if duration_sec is None:
            duration_sec = self.default_time

        # 构建轨迹
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9)
        )
        goal.trajectory.points = [point]

        self.logger.info(
            f'发送轨迹: {[f"{p:.2f}" for p in joint_positions]}, '
            f'时间: {duration_sec:.1f}s'
        )

        # 发送目标
        try:
            send_goal_future = self.action_client.send_goal_async(goal)

            # 等待目标被接受
            goal_handle: ClientGoalHandle = await asyncio.wait_for(
                asyncio.ensure_future(self._wait_for_future(send_goal_future)),
                timeout=5.0
            )

            if not goal_handle.accepted:
                self.logger.error('轨迹目标被拒绝')
                return False

            self.logger.info('轨迹目标已接受，等待执行完成...')

            # 等待执行完成
            result_future = goal_handle.get_result_async()
            result = await asyncio.wait_for(
                asyncio.ensure_future(self._wait_for_future(result_future)),
                timeout=duration_sec + 10.0
            )

            if result.result.error_code == 0:
                self.logger.info('轨迹执行成功')
                return True
            else:
                self.logger.error(f'轨迹执行失败，错误码: {result.result.error_code}')
                return False

        except asyncio.TimeoutError:
            self.logger.error('轨迹执行超时')
            return False
        except Exception as e:
            self.logger.error(f'轨迹执行异常: {e}')
            return False

    async def _wait_for_future(self, future):
        """等待 ROS2 Future 完成"""
        while not future.done():
            await asyncio.sleep(0.1)
            rclpy.spin_once(self.node, timeout_sec=0.01)
        return future.result()

    def move_to_joints_sync(
        self,
        joint_positions: List[float],
        duration_sec: float = None
    ) -> bool:
        """
        同步版本：移动到指定关节角度

        Args:
            joint_positions: 6 个关节角度 [rad]
            duration_sec: 运动时间（秒）

        Returns:
            是否成功
        """
        if len(joint_positions) != 6:
            self.logger.error(f'关节数量错误: {len(joint_positions)}, 需要 6 个')
            return False

        if duration_sec is None:
            duration_sec = self.default_time

        # 构建轨迹
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9)
        )
        goal.trajectory.points = [point]

        self.logger.info(
            f'发送轨迹: {[f"{p:.2f}" for p in joint_positions]}, '
            f'时间: {duration_sec:.1f}s'
        )

        # 发送目标并等待
        send_goal_future = self.action_client.send_goal_async(goal)

        # 等待目标被接受（使用简单循环，不使用 spin_until_future_complete）
        timeout = 5.0
        start_time = time.time()
        while not send_goal_future.done():
            if time.time() - start_time > timeout:
                self.logger.error('发送目标超时')
                return False
            time.sleep(0.01)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error('轨迹目标被拒绝')
            return False

        self.logger.info('轨迹目标已接受，等待执行完成...')

        # 等待执行完成（使用简单循环）
        result_future = goal_handle.get_result_async()
        timeout = duration_sec + 10.0
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                self.logger.error('轨迹执行超时')
                return False
            time.sleep(0.01)

        if not result_future.done():
            self.logger.error('轨迹执行超时')
            return False

        result = result_future.result()
        if result.result.error_code == 0:
            self.logger.info('轨迹执行成功')
            return True
        else:
            self.logger.error(f'轨迹执行失败，错误码: {result.result.error_code}')
            return False

    def is_ready(self) -> bool:
        """检查机械臂控制器是否就绪"""
        return (
            self.action_client.server_is_ready() and
            self.current_joints is not None
        )
