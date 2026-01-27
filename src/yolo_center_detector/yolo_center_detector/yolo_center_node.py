from __future__ import annotations
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D, Pose2D

from cv_bridge import CvBridge
import cv2  # noqa: F401
import numpy as np

from .detector import YoloCenterDetector


class YoloCenterNode(Node):
    def __init__(self):
        super().__init__("yolo_center_node")

        # ---- Parameters ----
        self.declare_parameter("model_path", "yolo11s.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("half", False)
        self.declare_parameter("target_list", ["person"])
        self.declare_parameter("topk_per_class", 0)  # 0=全部；>0=每类topk
        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("depth_topic", "/stereo/depth")  # 深度图话题
        self.declare_parameter("publish_pose_array", True)
        self.declare_parameter("publish_visualization", True)  # 发布带检测框的图像

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        device = self.get_parameter("device").get_parameter_value().string_value
        conf = self.get_parameter("conf").value
        iou = self.get_parameter("iou").value
        imgsz = int(self.get_parameter("imgsz").value)
        half = bool(self.get_parameter("half").value)

        self.target_list: List[str] = list(self.get_parameter("target_list").value)
        topk = int(self.get_parameter("topk_per_class").value)
        self.topk_per_class = None if topk <= 0 else topk

        self.get_logger().info(f"Loading model from: {model_path}")
        self.get_logger().info(f"Device: {device}")
        
        try:
            self.detector = YoloCenterDetector(
                model_path=model_path,
                device=device,
                conf=float(conf),
                iou=float(iou),
                imgsz=imgsz,
                half=half,
                verbose=False,
            )
            self.get_logger().info("Model loaded successfully!")
            
            # 显示模型支持的类名
            self.get_logger().info(f"Model supports {len(self.detector.id2name)} classes: {list(self.detector.id2name.values())}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise

        self.bridge = CvBridge()
        self.latest_depth = None  # 保存最新深度图
        self.depth_received = False  # 标记是否接收到深度图

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        
        # 图像订阅使用默认 sensor_data QoS (BEST_EFFORT)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        
        # 深度订阅使用 RELIABLE QoS 匹配发布者
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_depth = self.create_subscription(Image, depth_topic, self.on_depth, depth_qos)

        self.pub_det = self.create_publisher(Detection2DArray, "/detections", 10)
        self.pub_pose = self.create_publisher(PoseArray, "/centers", 10)
        self.pub_viz = self.create_publisher(Image, "/detections_visualization", 10)  # 发布带检测框的图像

        self.publish_pose_array = bool(self.get_parameter("publish_pose_array").value)
        self.publish_visualization = bool(self.get_parameter("publish_visualization").value)

        self.get_logger().info(f"Subscribed: {image_topic}")
        self.get_logger().info(f"Depth topic: {depth_topic}")
        self.get_logger().info(f"Target list: {self.target_list}")
        self.get_logger().info(f"Publishing visualizations: {self.publish_visualization}")

    def on_depth(self, msg: Image):
        """接收并存储深度图"""
        try:
            # 深度图通常是 16UC1 或 32FC1 格式
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if not self.depth_received:
                self.get_logger().info(f"Depth stream started: {self.latest_depth.shape}, dtype={self.latest_depth.dtype}, encoding={msg.encoding}")
                self.depth_received = True
            self.get_logger().debug(f"Depth image received: {self.latest_depth.shape}, dtype={self.latest_depth.dtype}, encoding={msg.encoding}")
        except Exception as e:
            self.get_logger().error(f"Depth image conversion error: {e}")

    def get_depth_at_bbox(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """获取检测框内的有效深度值（使用中位数，更鲁棒）"""
        if self.latest_depth is None:
            self.get_logger().debug("No depth image received yet")
            return 0.0
        
        h, w = self.latest_depth.shape[:2]
        x1 = max(0, min(w - 1, x1))
        y1 = max(0, min(h - 1, y1))
        x2 = max(0, min(w - 1, x2))
        y2 = max(0, min(h - 1, y2))
        
        # 获取检测框内的深度区域
        region = self.latest_depth[y1:y2+1, x1:x2+1]
        
        # 处理不同的深度格式
        if self.latest_depth.dtype == np.float32:
            # 32FC1: 通常已经是米为单位
            valid = region[(region > 0) & (region < 10.0)]  # 过滤无效和过大的值
            if len(valid) == 0:
                return 0.0
            return float(np.median(valid))
        else:
            # 16UC1: 通常以毫米为单位
            valid = region[region > 0]  # 过滤无效深度
            if len(valid) == 0:
                return 0.0
            depth_mm = float(np.median(valid))
            return depth_mm / 1000.0  # 转换为米

    def on_image(self, msg: Image):
        encoding = (msg.encoding or "bgr8").lower()

        try:
            # Keep original data first, then convert to BGR for detector/overlay
            raw_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # Ensure BGR for YOLO (and for consistent overlay colors)
        if encoding in ("rgb8", "rgba8"):
            img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR)
        elif encoding == "bgra8":
            img = cv2.cvtColor(raw_img, cv2.COLOR_BGRA2BGR)
        else:
            img = raw_img

        try:
            dets = self.detector.detect(img, self.target_list, topk_per_class=self.topk_per_class)
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return

        if len(dets) > 0:
            self.get_logger().info(f"Detected {len(dets)} objects from target list {self.target_list}")
            if self.latest_depth is not None:
                self.get_logger().debug(f"Depth image available: shape={self.latest_depth.shape}, dtype={self.latest_depth.dtype}")
            for det in dets:
                self.get_logger().debug(f"  - {det.cls_name}: center=({det.center[0]}, {det.center[1]}), conf={det.conf:.2f}")

        # ---- Publish Detection2DArray ----
        det_array = Detection2DArray()
        det_array.header = msg.header

        # ---- Prepare visualization image ----
        viz_img = img.copy() if self.publish_visualization else None

        for d in dets:
            x1, y1, x2, y2 = d.bbox
            cx, cy = d.center
            w = max(0.0, float(x2 - x1))
            h = max(0.0, float(y2 - y1))

            det_msg = Detection2D()
            det_msg.header = msg.header

            bbox = BoundingBox2D()
            pose2d = Pose2D()
            pose2d.position.x = float(cx)
            pose2d.position.y = float(cy)
            pose2d.theta = 0.0
            bbox.center = pose2d
            bbox.size_x = w
            bbox.size_y = h
            det_msg.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.cls_name
            hyp.hypothesis.score = float(d.conf)
            det_msg.results.append(hyp)

            det_array.detections.append(det_msg)

            # ---- Draw bbox on visualization ----
            if viz_img is not None:
                color = (0, 255, 0)  # 绿色
                thickness = 2
                # 绘制矩形框
                cv2.rectangle(viz_img, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
                # 绘制中心点
                cv2.circle(viz_img, (int(cx), int(cy)), 5, (0, 0, 255), -1)  # 红色圆点
                
                # 获取检测框内的深度值（使用整个框而不是中心点）
                depth = self.get_depth_at_bbox(int(x1), int(y1), int(x2), int(y2))
                self.get_logger().debug(f"Depth in bbox ({int(x1)}, {int(y1)})-({int(x2)}, {int(y2)}): {depth:.3f}m")
                if depth > 0:
                    label = f"{d.cls_name}: {d.conf:.2f} [{depth:.2f}m]"
                else:
                    label = f"{d.cls_name}: {d.conf:.2f}"
                cv2.putText(viz_img, label, (int(x1), int(y1) - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

        self.pub_det.publish(det_array)

        # ---- Publish visualization image ----
        if self.publish_visualization and viz_img is not None:
            try:
                # Match published encoding to original to avoid color shifts in viewers
                if encoding in ("rgb8", "rgba8"):
                    out_img = cv2.cvtColor(viz_img, cv2.COLOR_BGR2RGB)
                    out_enc = "rgb8"
                else:
                    out_img = viz_img
                    out_enc = "bgr8"
                viz_msg = self.bridge.cv2_to_imgmsg(out_img, encoding=out_enc)
                viz_msg.header = msg.header
                self.pub_viz.publish(viz_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish visualization: {e}")

        # ---- Optional Publish PoseArray of centers ----
        if self.publish_pose_array:
            pa = PoseArray()
            pa.header = msg.header
            for d in dets:
                cx, cy = d.center
                x1, y1, x2, y2 = d.bbox
                depth = self.get_depth_at_bbox(int(x1), int(y1), int(x2), int(y2))
                
                p = Pose()
                p.position.x = float(cx)
                p.position.y = float(cy)
                p.position.z = depth  # 深度值（米）
                p.orientation.w = 1.0
                pa.poses.append(p)
            self.pub_pose.publish(pa)


def main():
    rclpy.init()
    node = YoloCenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
