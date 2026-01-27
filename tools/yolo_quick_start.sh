#!/usr/bin/env bash
# yolo_center_detector 完整使用示例

echo "=========================================="
echo "YOLO Center Detector 使用指南"
echo "=========================================="
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 设置 ROS2 环境
source ~/ros2_ws/install/setup.bash

echo "✓ ROS2 环境已配置"
echo ""

# 检查包是否安装
ros2 pkg list | grep -q yolo_center_detector
if [ $? -ne 0 ]; then
    echo "✗ 错误：yolo_center_detector 包未安装"
    echo "  请先编译：cd ~/ros2_ws && colcon build --packages-select yolo_center_detector"
    exit 1
fi
echo "✓ yolo_center_detector 包已安装"
echo ""

# 检查模型文件
MODEL_PATH="/home/rvl/ros2_ws/src/yolo_center_detector/resource/models/yolo26s.pt"
if [ ! -f "$MODEL_PATH" ]; then
    echo "✗ 错误：模型文件不存在"
    echo "  路径：$MODEL_PATH"
    exit 1
fi
echo "✓ 模型文件已存在"
echo ""

# 显示可用图像话题
echo "=========================================="
echo "可用的图像话题："
echo "=========================================="
echo "1. /image                  - 通用图像话题"
echo "2. /color/video/image      - OAK-D RGB 图像（推荐）"
echo "3. /usb_cam/image_raw      - USB 摄像头"
echo "4. 自定义话题名称"
echo ""

# 提示用户选择
echo "=========================================="
echo "快速启动选项："
echo "=========================================="
echo ""
echo "选项 1：启动相机驱动（需要 OAK-D）"
echo "  ros2 launch depthai_examples rgb_stereo_node.launch.py"
echo ""
echo "选项 2：启动 YOLO 检测节点（需要先启动相机）"
echo "  ros2 launch yolo_center_detector yolo_center.launch.py"
echo ""
echo "选项 3：自定义图像话题启动"
echo "  ros2 run yolo_center_detector yolo_center_node \\"
echo "    --ros-args \\"
echo "    -p image_topic:=/color/video/image \\"
echo "    -p target_list:=\"[person,car,dog]\""
echo ""
echo "选项 4：查看可视化结果"
echo "  方法 A：使用 RViz2（推荐）"
echo "    rviz2"
echo "    # 添加 Image，设置话题为 /detections_visualization"
echo ""
echo "  方法 B：使用 image_tools"
echo "    ros2 run image_tools showimage --ros-args -r image:=/detections_visualization"
echo ""
echo "=========================================="
echo "话题监听示例："
echo "=========================================="
echo ""
echo "查看检测结果："
echo "  ros2 topic echo /detections"
echo ""
echo "查看中心点："
echo "  ros2 topic echo /centers"
echo ""
echo "查看带框图像发布情况："
echo "  ros2 topic info /detections_visualization -v"
echo ""
echo "=========================================="
echo "更多帮助："
echo "=========================================="
echo "查看详细说明文档："
echo "  cat /home/rvl/ros2_ws/05\ yolo识别.md"
echo ""
