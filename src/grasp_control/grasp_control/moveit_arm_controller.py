"""MoveIt2 机械臂控制模块：使用 MoveGroup Action 进行笛卡尔运动"""

from typing import List, Optional, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive


class MoveItArmController:
    """使用 MoveIt2 的机械臂控制器"""

    def __init__(self, node: Node, config: dict):
        self.node = node
        self.config = config
        self.logger = node.get_logger()

        # 配置参数
        moveit_cfg = config.get('moveit', {})
        self.planning_group = moveit_cfg.get('planning_group', 'ur_manipulator')
        self.base_frame = config.get('frames', {}).get('robot_base', 'base_link')
        self.ee_frame = moveit_cfg.get('end_effector_frame', 'tool0')
        self.planning_time = moveit_cfg.get('planning_time', 5.0)
        self.velocity_scaling = moveit_cfg.get('velocity_scaling', 0.3)
        self.acceleration_scaling = moveit_cfg.get('acceleration_scaling', 0.3)

        # MoveGroup Action 客户端
        self.action_client = ActionClient(
            node,
            MoveGroup,
            '/move_action'
        )

        self.logger.info(
            f'MoveIt 控制器初始化完成，规划组: {self.planning_group}'
        )

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """等待 MoveGroup Action 服务器就绪"""
        self.logger.info('等待 MoveIt2 服务器...')
        ready = self.action_client.wait_for_server(timeout_sec=timeout_sec)
        if ready:
            self.logger.info('MoveIt2 服务器已就绪')
        else:
            self.logger.error('MoveIt2 服务器连接超时')
        return ready

    def move_to_pose_sync(
        self,
        target_pose: Pose,
        velocity_scaling: float = None,
        acceleration_scaling: float = None
    ) -> bool:
        """
        同步移动到目标位姿

        Args:
            target_pose: 目标位姿 (geometry_msgs/Pose)
            velocity_scaling: 速度缩放因子 [0.0, 1.0]
            acceleration_scaling: 加速度缩放因子 [0.0, 1.0]

        Returns:
            是否成功
        """
        if velocity_scaling is None:
            velocity_scaling = self.velocity_scaling
        if acceleration_scaling is None:
            acceleration_scaling = self.acceleration_scaling

        # 构建 MoveGroup Goal
        goal = MoveGroup.Goal()
        goal.request = self._create_motion_plan_request(
            target_pose,
            velocity_scaling,
            acceleration_scaling
        )
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.logger.info(
            f'发送目标位姿: x={target_pose.position.x:.3f}, '
            f'y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}'
        )

        # 发送目标并等待
        send_goal_future = self.action_client.send_goal_async(goal)

        # 等待目标被接受
        timeout = 10.0
        try:
            rclpy.spin_until_future_complete(
                self.node,
                send_goal_future,
                timeout_sec=timeout
            )
        except Exception as e:
            self.logger.error(f'发送目标异常: {e}')
            return False

        if not send_goal_future.done():
            self.logger.error('发送目标超时')
            return False

        goal_handle: ClientGoalHandle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error('运动规划目标被拒绝')
            return False

        self.logger.info('目标已接受，等待规划和执行...')

        # 等待执行完成
        result_future = goal_handle.get_result_async()
        timeout = self.planning_time + 30.0  # 规划时间 + 执行时间
        try:
            rclpy.spin_until_future_complete(
                self.node,
                result_future,
                timeout_sec=timeout
            )
        except Exception as e:
            self.logger.error(f'运动执行异常: {e}')
            return False

        if not result_future.done():
            self.logger.error('运动执行超时')
            return False

        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.logger.info('运动执行成功')
            return True
        else:
            self.logger.error(
                f'运动执行失败，错误码: {result.result.error_code.val}'
            )
            return False

    def _create_motion_plan_request(
        self,
        target_pose: Pose,
        velocity_scaling: float,
        acceleration_scaling: float
    ) -> MotionPlanRequest:
        """创建运动规划请求"""
        request = MotionPlanRequest()

        # 基本设置
        request.group_name = self.planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling

        # 目标约束
        goal_constraints = Constraints()

        # 位置约束
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.ee_frame
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # 约束区域（小球体）
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]  # 1mm 精度
        bounding_volume.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position = target_pose.position
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        goal_constraints.position_constraints.append(position_constraint)

        # 姿态约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.ee_frame
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        goal_constraints.orientation_constraints.append(orientation_constraint)

        request.goal_constraints.append(goal_constraints)

        return request

    def is_ready(self) -> bool:
        """检查 MoveIt2 是否就绪"""
        return self.action_client.server_is_ready()
