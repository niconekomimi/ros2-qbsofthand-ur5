from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 获取包的安装路径
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
                "conf": 0.25,
                "iou": 0.7,
                "imgsz": 640,
                "half": False,
                "target_list": ["person", "bottle", "cup", "orange"],
                "topk_per_class": 0,
                "image_topic": "/image",
                "depth_topic": "/stereo/depth",
                "publish_pose_array": True,
            }],
        )
    ])
