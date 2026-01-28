"""坐标变换模块：像素坐标 → 相机坐标 → 机械臂基座坐标"""

import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PointStamped, Pose, TransformStamped
from sensor_msgs.msg import CameraInfo
import tf2_geometry_msgs


class CoordinateTransformer:
    """坐标变换器：将像素坐标转换为机械臂基座坐标"""

    def __init__(self, node: Node, config: dict):
        self.node = node
        self.config = config
        self.logger = node.get_logger()

        # TF2 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        # 相机内参（从配置加载默认值）
        cam_cfg = config.get('camera', {})
        self.fx = cam_cfg.get('fx', 615.0)
        self.fy = cam_cfg.get('fy', 615.0)
        self.cx = cam_cfg.get('cx', 320.0)
        self.cy = cam_cfg.get('cy', 240.0)
        self.camera_info_received = False

        # 坐标系名称
        frames = config.get('frames', {})
        self.base_frame = frames.get('robot_base', 'base_link')
        self.effector_frame = frames.get('robot_effector', 'tool0')
        self.camera_frame = frames.get('camera', 'oak_rgb_camera_optical_frame')

        # 订阅相机内参
        info_topic = cam_cfg.get('info_topic', '/color/video/camera_info')
        self.camera_info_sub = node.create_subscription(
            CameraInfo, info_topic, self._camera_info_callback, 10
        )
        self.logger.info(f'坐标变换器初始化完成，等待相机内参: {info_topic}')

    def _camera_info_callback(self, msg: CameraInfo):
        """更新相机内参"""
        if not self.camera_info_received:
            # K 矩阵: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.logger.info(
                f'相机内参已更新: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}'
            )

    def pixel_to_camera(self, u: float, v: float, depth: float) -> np.ndarray:
        """
        像素坐标 + 深度 → 相机坐标系 3D 点

        Args:
            u: 像素 x 坐标
            v: 像素 y 坐标
            depth: 深度值（米）

        Returns:
            相机坐标系中的 3D 点 [x, y, z]
        """
        x_cam = (u - self.cx) * depth / self.fx
        y_cam = (v - self.cy) * depth / self.fy
        z_cam = depth
        return np.array([x_cam, y_cam, z_cam])

    def camera_to_base(self, point_camera: np.ndarray) -> Optional[np.ndarray]:
        """
        相机坐标系 → 机械臂基座坐标系

        Args:
            point_camera: 相机坐标系中的 3D 点 [x, y, z]

        Returns:
            基座坐标系中的 3D 点 [x, y, z]，失败返回 None
        """
        # 创建 PointStamped 消息
        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.header.stamp = self.node.get_clock().now().to_msg()
        point_stamped.point.x = float(point_camera[0])
        point_stamped.point.y = float(point_camera[1])
        point_stamped.point.z = float(point_camera[2])

        try:
            # 等待 TF 变换可用
            self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 执行坐标转换
            point_base = self.tf_buffer.transform(point_stamped, self.base_frame)

            return np.array([
                point_base.point.x,
                point_base.point.y,
                point_base.point.z
            ])

        except TransformException as e:
            self.logger.error(f'TF 变换失败: {e}')
            return None

    def pixel_to_base(self, u: float, v: float, depth: float) -> Optional[np.ndarray]:
        """
        完整变换：像素坐标 + 深度 → 机械臂基座坐标

        Args:
            u: 像素 x 坐标
            v: 像素 y 坐标
            depth: 深度值（米）

        Returns:
            基座坐标系中的 3D 点 [x, y, z]，失败返回 None
        """
        if depth <= 0:
            self.logger.warn(f'无效深度值: {depth}')
            return None

        # 像素 → 相机坐标
        point_camera = self.pixel_to_camera(u, v, depth)
        self.logger.debug(
            f'像素({u:.1f}, {v:.1f}, d={depth:.3f}m) → '
            f'相机({point_camera[0]:.3f}, {point_camera[1]:.3f}, {point_camera[2]:.3f})'
        )

        # 相机 → 基座坐标
        point_base = self.camera_to_base(point_camera)
        if point_base is not None:
            self.logger.debug(
                f'→ 基座({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f})'
            )

        return point_base

    def compute_grasp_pose(
        self,
        target_position: np.ndarray,
        approach_height: float,
        orientation_rpy: Tuple[float, float, float]
    ) -> Pose:
        """
        计算抓取位姿

        Args:
            target_position: 目标位置 [x, y, z]（基座坐标系）
            approach_height: 接近高度（物体上方，米）
            orientation_rpy: 末端姿态 (roll, pitch, yaw) 弧度

        Returns:
            geometry_msgs/Pose 消息
        """
        pose = Pose()

        # 位置：物体上方
        pose.position.x = float(target_position[0])
        pose.position.y = float(target_position[1])
        pose.position.z = float(target_position[2] + approach_height)

        # 姿态：RPY → 四元数
        roll, pitch, yaw = orientation_rpy
        q = self._rpy_to_quaternion(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """RPY 欧拉角转四元数 (x, y, z, w)"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([x, y, z, w])

    def is_ready(self) -> bool:
        """检查坐标变换器是否就绪"""
        # 检查 TF 是否可用
        try:
            self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return True
        except TransformException:
            return False
