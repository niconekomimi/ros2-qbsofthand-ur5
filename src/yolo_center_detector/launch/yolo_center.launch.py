from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_dir = "/home/rvl/ros2_ws/src/yolo_center_detector"
    model_path = os.path.join(package_dir, "resource/models/yolo26s.pt")
    
    return LaunchDescription([
        Node(
            package="yolo_center_detector",
            executable="yolo_center_node",
            name="yolo_center_node",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "device": "cuda:0",
                "conf": 0.3,
                "iou": 0.7,
                "imgsz": 640,
                "half": False,
                "target_list": ["orange"],  # 检测橙子
                "topk_per_class": 0,
                
                "image_topic": "/oak/rgb/image_raw",
                "depth_topic": "/oak/stereo/image_raw",

                # 0.4 表示只取检测框中心 40% 的区域测距，非常能抗背景干扰
                "depth_roi_ratio": 0.4, 
                
                "publish_pose_array": True,
                "publish_visualization": True,
            }],
        )
    ])
