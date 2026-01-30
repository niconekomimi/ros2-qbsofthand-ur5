"""坐标变换模块：像素坐标 → 相机坐标 → 机械臂基座坐标"""

import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PointStamped, Pose
from sensor_msgs.msg import CameraInfo

# [关键] 必须导入，否则报错 Type is not loaded
import tf2_geometry_msgs 


class CoordinateTransformer:
    """坐标变换器"""

    def __init__(self, node: Node, config: dict):
        self.node = node
        self.config = config
        self.logger = node.get_logger()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        cam_cfg = config.get('camera', {})
        self.fx = cam_cfg.get('fx', 615.0)
        self.fy = cam_cfg.get('fy', 615.0)
        self.cx = cam_cfg.get('cx', 320.0)
        self.cy = cam_cfg.get('cy', 240.0)
        self.camera_info_received = False

        # [修正] 读取坐标系配置
        frames = config.get('frames', {})
        self.base_frame = frames.get('robot_base', 'base')
        self.camera_frame = frames.get('camera', 'oak_rgb_camera_optical_frame')
        # [补全] 必须初始化 effector_frame，否则 get_current_tcp_pose 会报错
        self.effector_frame = frames.get('robot_effector', 'tool0')

        info_topic = cam_cfg.get('info_topic', '/color/video/camera_info')
        self.camera_info_sub = node.create_subscription(
            CameraInfo, info_topic, self._camera_info_callback, 10
        )

    def _camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True

    def pixel_to_camera(self, u: float, v: float, depth: float) -> np.ndarray:
        x_cam = (u - self.cx) * depth / self.fx
        y_cam = (v - self.cy) * depth / self.fy
        z_cam = depth
        return np.array([x_cam, y_cam, z_cam])

    def camera_to_base(self, point_camera: np.ndarray) -> Optional[np.ndarray]:
        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        point_stamped.point.x = float(point_camera[0])
        point_stamped.point.y = float(point_camera[1])
        point_stamped.point.z = float(point_camera[2])

        try:
            self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            point_base = self.tf_buffer.transform(point_stamped, self.base_frame)
            return np.array([point_base.point.x, point_base.point.y, point_base.point.z])
        except TransformException as e:
            self.logger.error(f'TF 变换失败: {e}')
            return None

    def pixel_to_base(self, u: float, v: float, depth: float) -> Optional[np.ndarray]:
        if depth <= 0: return None
        point_camera = self.pixel_to_camera(u, v, depth)
        point_base = self.camera_to_base(point_camera)
        return point_base

    def get_current_tcp_pose(self) -> Optional[Pose]:
        """[新增] 获取当前末端执行器在基座下的位姿"""
        try:
            # 获取最新变换 (base -> tool0)
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.effector_frame,  # 这里现在可以正常读取了
                rclpy.time.Time()
            )
            
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except TransformException as e:
            self.logger.error(f'无法获取当前机械臂位置: {e}')
            return None

    def compute_grasp_pose(
        self,
        target_position: np.ndarray,
        orientation_q: np.ndarray
    ) -> Pose:
        """
        计算抓取位姿
        Args:
            target_position: 目标位置 [x, y, z]
            orientation_q: 目标姿态四元数 [x, y, z, w]
        """
        pose = Pose()

        # 读取 config 中的 tcp_offset
        grasp_cfg = self.config.get('grasp', {})
        tcp_offset = grasp_cfg.get('tcp_offset', {})
        off_x = tcp_offset.get('x', 0.0)
        off_y = tcp_offset.get('y', 0.0)
        off_z = tcp_offset.get('z', 0.0)

        # 叠加偏移量
        pose.position.x = float(target_position[0]) + off_x
        pose.position.y = float(target_position[1]) + off_y
        pose.position.z = float(target_position[2]) + off_z

        # 设置姿态
        pose.orientation.x = float(orientation_q[0])
        pose.orientation.y = float(orientation_q[1])
        pose.orientation.z = float(orientation_q[2])
        pose.orientation.w = float(orientation_q[3])

        return pose

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
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

    @staticmethod
    def _rotvec_to_quaternion(rx: float, ry: float, rz: float) -> np.ndarray:
        """旋转向量 (rx, ry, rz) 转四元数"""
        angle = np.sqrt(rx**2 + ry**2 + rz**2)
        if angle < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0])
        
        axis = np.array([rx, ry, rz]) / angle
        sin_half = np.sin(angle / 2.0)
        cos_half = np.cos(angle / 2.0)
        
        x = axis[0] * sin_half
        y = axis[1] * sin_half
        z = axis[2] * sin_half
        w = cos_half
        return np.array([x, y, z, w])

    def is_ready(self) -> bool:
        try:
            self.tf_buffer.can_transform(self.base_frame, self.camera_frame, Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            return True
        except TransformException:
            return False