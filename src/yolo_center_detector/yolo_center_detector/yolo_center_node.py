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
        self.declare_parameter("topk_per_class", 0)
        
        # [修改] 更新为 OAK-D 默认话题
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("depth_topic", "/oak/stereo/image_raw")
        
        # [关键] 深度采样区域比例 (0.4 表示只取中心 40% 的区域测距，避免背景干扰)
        self.declare_parameter("depth_roi_ratio", 0.4) 
        
        self.declare_parameter("publish_pose_array", True)
        self.declare_parameter("publish_visualization", True)

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
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise

        self.bridge = CvBridge()
        self.latest_depth = None
        self.depth_received = False

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        
        self.sub = self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_depth = self.create_subscription(Image, depth_topic, self.on_depth, depth_qos)

        self.pub_det = self.create_publisher(Detection2DArray, "/detections", 10)
        self.pub_pose = self.create_publisher(PoseArray, "/centers", 10)
        self.pub_viz = self.create_publisher(Image, "/detections_visualization", 10)

        self.publish_pose_array = bool(self.get_parameter("publish_pose_array").value)
        self.publish_visualization = bool(self.get_parameter("publish_visualization").value)
        self.depth_roi_ratio = float(self.get_parameter("depth_roi_ratio").value)

        self.get_logger().info(f"Subscribed Image: {image_topic}")
        self.get_logger().info(f"Subscribed Depth: {depth_topic}")
        self.get_logger().info(f"Depth ROI Ratio: {self.depth_roi_ratio} (Center Sampling)")

    def on_depth(self, msg: Image):
        """接收并存储深度图"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if not self.depth_received:
                self.get_logger().info(f"Depth stream started: {self.latest_depth.shape}, dtype={self.latest_depth.dtype}")
                self.depth_received = True
        except Exception as e:
            self.get_logger().error(f"Depth image conversion error: {e}")

    def get_central_roi(self, x1: int, y1: int, x2: int, y2: int) -> tuple[int, int, int, int]:
        """
        计算检测框中心的采样区域 (ROI)
        注意：这里假设 RGB 和 深度图已经开启了 align_depth，分辨率一致，不需要坐标映射
        """
        if self.latest_depth is None:
            return x1, y1, x2, y2

        h_img, w_img = self.latest_depth.shape[:2]

        box_w = x2 - x1
        box_h = y2 - y1
        cx = x1 + box_w / 2
        cy = y1 + box_h / 2

        # 计算 ROI 大小
        roi_w = box_w * self.depth_roi_ratio
        roi_h = box_h * self.depth_roi_ratio

        rx1 = int(cx - roi_w / 2)
        rx2 = int(cx + roi_w / 2)
        ry1 = int(cy - roi_h / 2)
        ry2 = int(cy + roi_h / 2)

        # 边界限制
        rx1 = max(0, min(w_img - 1, rx1))
        ry1 = max(0, min(h_img - 1, ry1))
        rx2 = max(0, min(w_img - 1, rx2))
        ry2 = max(0, min(h_img - 1, ry2))

        return rx1, ry1, rx2, ry2

    def get_depth_at_bbox(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """获取检测框内的有效深度值"""
        if self.latest_depth is None:
            return 0.0

        # 获取中心 ROI 区域（避开背景）
        rx1, ry1, rx2, ry2 = self.get_central_roi(x1, y1, x2, y2)

        if rx2 <= rx1 or ry2 <= ry1:
            return 0.0
        
        region = self.latest_depth[ry1:ry2, rx1:rx2]
        
        # 32FC1 (Meter)
        if self.latest_depth.dtype == np.float32:
            valid = region[(region > 0.1) & (region < 15.0)] # 过滤 0 和过远
            if len(valid) == 0: return 0.0
            return float(np.median(valid))
        
        # 16UC1 (Millimeter)
        else:
            valid = region[region > 0]
            if len(valid) == 0: return 0.0
            depth_mm = float(np.median(valid))
            return depth_mm / 1000.0

    def on_image(self, msg: Image):
        # 1. Image Processing
        encoding = (msg.encoding or "bgr8").lower()
        try:
            raw_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        if encoding in ("rgb8", "rgba8"):
            img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR)
        elif encoding == "bgra8":
            img = cv2.cvtColor(raw_img, cv2.COLOR_BGRA2BGR)
        else:
            img = raw_img

        # 2. Inference
        try:
            dets = self.detector.detect(img, self.target_list, topk_per_class=self.topk_per_class)
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return

        # 3. Process Detections
        det_array = Detection2DArray()
        det_array.header = msg.header
        
        # [修改] 准备可视化图像：默认为 RGB
        viz_img = img.copy() if self.publish_visualization else None
        
        # [新增] 深度图融合调试：如果有深度图，将其彩色化并叠加到 viz_img 上
        if self.publish_visualization and viz_img is not None and self.latest_depth is not None:
            try:
                # 获取深度图并归一化到 0-255 用于显示
                # 限制范围 0-5米 以获得更好的对比度
                depth_disp = self.latest_depth.copy()
                if self.latest_depth.dtype == np.float32:
                    depth_disp = np.clip(depth_disp, 0, 5.0) 
                    depth_disp = cv2.normalize(depth_disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                else:
                    # 16UC1 (mm) -> clip at 5000mm
                    depth_disp = np.clip(depth_disp, 0, 5000)
                    depth_disp = cv2.normalize(depth_disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                
                # 生成热力图 (近=红/暖, 远=蓝/冷)
                depth_colormap = cv2.applyColorMap(depth_disp, cv2.COLORMAP_JET)
                
                # 如果尺寸不一致（说明没对齐），强行缩放以便观察偏离情况
                if depth_colormap.shape[:2] != viz_img.shape[:2]:
                    depth_colormap = cv2.resize(depth_colormap, (viz_img.shape[1], viz_img.shape[0]))
                
                # 叠加：RGB * 0.6 + Depth * 0.4
                viz_img = cv2.addWeighted(viz_img, 0.6, depth_colormap, 0.4, 0)
                
                cv2.putText(viz_img, "Depth Overlay Mode", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            except Exception as e:
                self.get_logger().warn(f"Visualization overlay error: {e}")

        for d in dets:
            x1, y1, x2, y2 = d.bbox
            cx, cy = d.center
            w, h = float(x2 - x1), float(y2 - y1)

            # 获取深度
            depth = self.get_depth_at_bbox(int(x1), int(y1), int(x2), int(y2))

            det_msg = Detection2D()
            det_msg.header = msg.header
            bbox = BoundingBox2D()
            pose2d = Pose2D()
            pose2d.position.x = float(cx); pose2d.position.y = float(cy)
            bbox.center = pose2d; bbox.size_x = w; bbox.size_y = h
            det_msg.bbox = bbox
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.cls_name
            hyp.hypothesis.score = float(d.conf)
            det_msg.results.append(hyp)
            det_array.detections.append(det_msg)

            # Draw Visualization
            if viz_img is not None:
                # 绿色框：YOLO
                cv2.rectangle(viz_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                # 蓝色框：深度采样区
                rx1, ry1, rx2, ry2 = self.get_central_roi(int(x1), int(y1), int(x2), int(y2))
                cv2.rectangle(viz_img, (rx1, ry1), (rx2, ry2), (255, 0, 0), 2)
                
                label_text = f"{d.cls_name} {depth:.2f}m"
                # 加个黑色背景让字更清楚
                t_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                cv2.rectangle(viz_img, (int(x1), int(y1)-25), (int(x1)+t_size[0], int(y1)), (0,0,0), -1)
                cv2.putText(viz_img, label_text, (int(x1), int(y1)-5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        self.pub_det.publish(det_array)

        # Publish Viz
        if self.publish_visualization and viz_img is not None:
            try:
                # 强制转换为 RGB8 格式发布，解决变色问题
                # OpenCV 内部是 BGR，ROS 里的显示工具通常期望 RGB
                out_img = cv2.cvtColor(viz_img, cv2.COLOR_BGR2RGB)
                
                viz_msg = self.bridge.cv2_to_imgmsg(out_img, encoding="rgb8")
                viz_msg.header = msg.header
                self.pub_viz.publish(viz_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish visualization: {e}")
        
        # Publish Pose
        if self.publish_pose_array:
            pa = PoseArray()
            pa.header = msg.header
            for d in dets:
                cx, cy = d.center
                x1, y1, x2, y2 = d.bbox
                depth = self.get_depth_at_bbox(int(x1), int(y1), int(x2), int(y2))
                p = Pose()
                p.position.x = float(cx); p.position.y = float(cy); p.position.z = depth
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