"""坐标变换模块：像素坐标 → 相机坐标 → 机械臂基座坐标 (Eye-in-Hand 鲁棒版)"""

import numpy as np
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PointStamped, Pose
from sensor_msgs.msg import CameraInfo

# [关键] 必须导入，否则报错 Type is not loaded
import tf2_geometry_msgs 


class CoordinateTransformer:
    """坐标变换器 (支持 Eye-in-Hand 自动回退计算)"""

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
        # [补全] Eye-in-Hand 必须要有末端坐标系
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
        """
        Eye-in-Hand 核心逻辑：
        1. 优先尝试 TF (如果有外部标定节点发布 Tool0->Camera)
        2. 失败则使用 Config 参数手动计算 (Base->Tool0[TF] * Tool0->Camera[Config])
        """
        
        # --- 方案 A: 尝试 TF 直接变换 ---
        try:
            # 快速检查 (0.05s)，不通则立刻切换到手动计算
            if self.tf_buffer.can_transform(self.base_frame, self.camera_frame, Time(), timeout=rclpy.duration.Duration(seconds=0.05)):
                pt = PointStamped()
                pt.header.frame_id = self.camera_frame
                pt.header.stamp = Time().to_msg()
                pt.point.x, pt.point.y, pt.point.z = float(point_camera[0]), float(point_camera[1]), float(point_camera[2])
                
                res = self.tf_buffer.transform(pt, self.base_frame)
                return np.array([res.point.x, res.point.y, res.point.z])
        except TransformException:
            pass # 进入手动模式

        # --- 方案 B: 手动 Eye-in-Hand 计算 ---
        try:
            # 1. 必须从 TF 获取机械臂实时位姿 (Base -> Tool0)
            t_robot = self.tf_buffer.lookup_transform(self.base_frame, self.effector_frame, Time())
            
            # 2. 从 Config 获取静态手眼参数 (Tool0 -> Camera)
            handeye = self.config.get('handeye', {})
            cfg_xyz = handeye.get('xyz')
            cfg_quat = handeye.get('rotation_quaternion')

            if not (cfg_xyz and cfg_quat):
                self.logger.error("TF断连且Config缺少handeye参数")
                return None

            # --- 链式计算: P_base = T_robot * T_handeye * P_cam ---
            
            # Step 1: Camera -> Tool0 (使用 Config)
            p_cam_vec = point_camera
            R_cfg = self._quat_to_mat(cfg_quat)
            T_cfg = np.array(cfg_xyz)
            p_tool = R_cfg @ p_cam_vec + T_cfg

            # Step 2: Tool0 -> Base (使用 TF 实时数据)
            tx = t_robot.transform.translation.x
            ty = t_robot.transform.translation.y
            tz = t_robot.transform.translation.z
            rx = t_robot.transform.rotation.x
            ry = t_robot.transform.rotation.y
            rz = t_robot.transform.rotation.z
            rw = t_robot.transform.rotation.w
            
            R_robot = self._quat_to_mat([rx, ry, rz, rw])
            T_robot = np.array([tx, ty, tz])
            
            p_base = R_robot @ p_tool + T_robot
            
            return p_base

        except TransformException:
            self.logger.error("无法获取机械臂位姿 (Base->Tool0)，请检查驱动")
            return None
        except Exception as e:
            self.logger.error(f"计算异常: {e}")
            return None

    def pixel_to_base(self, u: float, v: float, depth: float) -> Optional[np.ndarray]:
        if depth <= 0: return None
        point_camera = self.pixel_to_camera(u, v, depth)
        point_base = self.camera_to_base(point_camera)
        return point_base

    def get_current_tcp_pose(self) -> Optional[Pose]:
        """获取当前末端执行器在基座下的位姿"""
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.effector_frame,
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
        pose = Pose()
        grasp_cfg = self.config.get('grasp', {})
        tcp_offset = grasp_cfg.get('tcp_offset', {})
        
        pose.position.x = float(target_position[0]) + tcp_offset.get('x', 0.0)
        pose.position.y = float(target_position[1]) + tcp_offset.get('y', 0.0)
        pose.position.z = float(target_position[2]) + tcp_offset.get('z', 0.0)

        pose.orientation.x = float(orientation_q[0])
        pose.orientation.y = float(orientation_q[1])
        pose.orientation.z = float(orientation_q[2])
        pose.orientation.w = float(orientation_q[3])
        return pose

    @staticmethod
    def _quat_to_mat(q: List[float]) -> np.ndarray:
        """[新增] 辅助函数：四元数 [x, y, z, w] 转 3x3 旋转矩阵"""
        x, y, z, w = q
        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        return np.array([
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ])

    @staticmethod
    def _rotvec_to_quaternion(rx: float, ry: float, rz: float) -> np.ndarray:
        angle = np.sqrt(rx**2 + ry**2 + rz**2)
        if angle < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0])
        axis = np.array([rx, ry, rz]) / angle
        s, c = np.sin(angle / 2.0), np.cos(angle / 2.0)
        return np.array([axis[0]*s, axis[1]*s, axis[2]*s, c])

    def is_ready(self) -> bool:
        """检查 TF 是否就绪 (只要机械臂在线即可)"""
        try:
            # 只要能查到 Base -> Tool0，说明机械臂驱动正常
            # 相机参数可以直接读 config，所以只要机械臂在线就算 Ready
            return self.tf_buffer.can_transform(self.base_frame, self.effector_frame, Time(), timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException:
            return False