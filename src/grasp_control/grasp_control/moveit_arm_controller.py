"""MoveIt2 机械臂控制模块：支持关节角度约束"""

import time
import math
from typing import List, Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

# 必须导入 JointConstraint
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint,
    OrientationConstraint, JointConstraint, BoundingVolume, RobotTrajectory
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration


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
        self.velocity_scaling = moveit_cfg.get('velocity_scaling', 0.1)
        self.acceleration_scaling = moveit_cfg.get('acceleration_scaling', 0.1)

        # Action 客户端
        self.action_client = ActionClient(node, MoveGroup, '/move_action')
        
        # 笛卡尔路径服务
        self.cartesian_client = node.create_client(
            GetCartesianPath, '/compute_cartesian_path'
        )

        self.logger.info(f'MoveIt 控制器初始化完成，规划组: {self.planning_group}')

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        self.logger.info('等待 MoveIt2 服务器...')
        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.logger.error('MoveAction 连接超时')
            return False
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout_sec):
            self.logger.error('CartesianPath 服务连接超时')
            return False
        self.logger.info('MoveIt2 服务器已就绪')
        return True

    def move_cartesian_path(self, waypoints: List[Pose], step: float = 0.01, speed_factor: float = 0.1) -> bool:
        """执行笛卡尔直线运动 (带手动降速)"""
        if not self.cartesian_client.service_is_ready():
            self.logger.error('无法连接到 compute_cartesian_path 服务')
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = self.planning_group
        req.waypoints = waypoints
        req.max_step = step
        req.jump_threshold = 0.0 
        req.avoid_collisions = True

        future = self.cartesian_client.call_async(req)
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        response = future.result()

        if response.error_code.val != 1:
            self.logger.error(f'笛卡尔路径计算失败: {response.error_code.val}')
            return False

        if response.fraction < 0.90:
             self.logger.warn(f'笛卡尔路径仅计算了 {response.fraction*100:.1f}%，放弃执行')
             return False

        # 手动降速
        slow_trajectory = self._scale_trajectory_speed(response.solution, speed_factor)
        return self._execute_trajectory(slow_trajectory)

    def _scale_trajectory_speed(self, robot_traj: RobotTrajectory, scale: float) -> RobotTrajectory:
        """手动重新计算时间戳以降低速度"""
        joint_traj = robot_traj.joint_trajectory
        if not joint_traj.points: return robot_traj

        base_speed = 1.5 * scale  # 基准速度 (rad/s)
        
        new_points = []
        current_time = 0.0
        
        point0 = joint_traj.points[0]
        point0.time_from_start = Duration(sec=0, nanosec=0)
        new_points.append(point0)

        for i in range(1, len(joint_traj.points)):
            prev_p = joint_traj.points[i-1]
            curr_p = joint_traj.points[i]
            
            max_delta = 0.0
            for j in range(len(curr_p.positions)):
                delta = abs(curr_p.positions[j] - prev_p.positions[j])
                max_delta = max(max_delta, delta)
            
            dt = max_delta / base_speed
            dt = max(dt, 0.05) # 最小时间间隔

            current_time += dt
            
            sec = int(current_time)
            nanosec = int((current_time - sec) * 1e9)
            curr_p.time_from_start = Duration(sec=sec, nanosec=nanosec)
            curr_p.velocities = []
            curr_p.accelerations = []
            curr_p.effort = []
            new_points.append(curr_p)

        robot_traj.joint_trajectory.points = new_points
        return robot_traj

    def _execute_trajectory(self, robot_trajectory) -> bool:
        if not hasattr(self, 'exec_traj_client'):
            self.exec_traj_client = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')
        
        if not self.exec_traj_client.wait_for_server(timeout_sec=2.0): return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_trajectory
        
        self.logger.info('开始执行轨迹...')
        future = self.exec_traj_client.send_goal_async(goal)
        while not future.done(): rclpy.spin_once(self.node, timeout_sec=0.01)
        goal_handle = future.result()
        if not goal_handle.accepted: return False
            
        res_future = goal_handle.get_result_async()
        while not res_future.done(): rclpy.spin_once(self.node, timeout_sec=0.01)
        return res_future.result().result.error_code.val == 1

    def move_to_pose_sync(
        self, 
        target_pose: Pose, 
        velocity_scaling=None, 
        joint_limits: Optional[Dict[str, tuple]] = None
    ) -> bool:
        """
        PTP 移动，支持关节约束
        Args:
            joint_limits: 字典 {'joint_name': (min_angle, max_angle)}
        """
        if velocity_scaling is None: velocity_scaling = self.velocity_scaling

        goal = MoveGroup.Goal()
        goal.request = self._create_motion_plan_request(
            target_pose, velocity_scaling, self.acceleration_scaling, joint_limits
        )
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        
        self.logger.info('规划路径 (Smart PTP)...')
        future = self.action_client.send_goal_async(goal)
        while not future.done(): rclpy.spin_once(self.node, timeout_sec=0.01)
        goal_handle = future.result()
        if not goal_handle.accepted: return False
        
        res_future = goal_handle.get_result_async()
        while not res_future.done(): rclpy.spin_once(self.node, timeout_sec=0.01)
        return res_future.result().result.error_code.val == 1

    def _create_motion_plan_request(
        self, target_pose, v_scale, a_scale, 
        joint_limits: Optional[Dict[str, tuple]] = None
    ):
        req = MotionPlanRequest()
        req.group_name = self.planning_group
        req.max_velocity_scaling_factor = v_scale
        req.max_acceleration_scaling_factor = a_scale
        
        c = Constraints()
        
        # 1. 位置约束
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_frame
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.001]
        bv.primitives.append(sp)
        p = Pose(); p.position = target_pose.position; p.orientation.w=1.0
        bv.primitive_poses.append(p)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)
        
        # 2. 姿态约束
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_frame
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        # 3. [新增] 关节约束
        if joint_limits:
            for name, (min_val, max_val) in joint_limits.items():
                jc = JointConstraint()
                jc.joint_name = name
                # 目标位置设为区间中点
                jc.position = (min_val + max_val) / 2.0
                # 容差设为区间的一半
                jc.tolerance_above = (max_val - min_val) / 2.0
                jc.tolerance_below = (max_val - min_val) / 2.0
                jc.weight = 1.0
                c.joint_constraints.append(jc)
        
        req.goal_constraints.append(c)
        return req

    def is_ready(self) -> bool:
        return self.action_client.server_is_ready()