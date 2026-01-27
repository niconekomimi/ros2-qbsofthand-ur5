# YOLO 物体识别

## 快速开始

### 1. 编译（首次使用）
```bash
cd ~/ros2_ws
colcon build --packages-select yolo_center_detector --symlink-install
source install/setup.bash
```

### 2. 启动相机（OAK-D）
```bash
ros2 launch depthai_examples rgb_stereo_node.launch.py
```

### 3. 启动 YOLO 检测
```bash
source ~/ros2_ws/install/setup.bash

# 方式1：使用 launch 文件（检测 person, bottle, cup, orange）
ros2 launch yolo_center_detector yolo_center.launch.py

# 方式2：自定义参数（带深度信息）
ros2 run yolo_center_detector yolo_center_node \
  --ros-args \
  -p model_path:=/home/rvl/ros2_ws/src/yolo_center_detector/resource/models/yolo26s.pt \
  -p image_topic:=/color/video/image \
  -p depth_topic:=/stereo/depth \
  -p target_list:="[orange,person,bottle,cup]" \
  -p conf:=0.3
```

### 4. 查看检测结果
```bash
# 可视化图像（带框、中心点和深度）
ros2 run image_tools showimage --ros-args -r image:=/detections_visualization

# 查看中心点坐标+深度（position.z 为深度值，单位米）
ros2 topic echo /centers

# 查看详细检测信息
ros2 topic echo /detections
```

---

## 常用参数

| 参数           |  默认值          |  说明                                     |
|---------------|-----------------|-------------------------------------------|
| `image_topic` | /image          | 输入图像话题（OAK-D 用 `/color/video/image`）|
| `depth_topic` | /stereo/depth   | 深度图话题（OAK-D 深度图）                   |
| `target_list` | ["person"]      | 要检测的物体列表                             |
| `conf`        | 0.25            | 置信度阈值（0-1）                           |
| `device`      | cuda:0          | 推理设备（cuda:0 或 cpu）                   |

**支持检测的物体（COCO 80类）：**
```
person, bicycle, car, motorcycle, bus, truck, bottle, cup, fork, knife, spoon, bowl, 
banana, apple, sandwich, orange, broccoli, carrot, pizza, cake, chair, couch, bed, 
dining table, tv, laptop, mouse, keyboard, cell phone, book, clock, vase, scissors, ...
```
<details>
<summary>查看完整列表</summary>

person, bicycle, car, motorcycle, airplane, bus, train, truck, boat, traffic light, fire hydrant, stop sign, parking meter, bench, bird, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack, umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket, bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake, chair, couch, potted plant, bed, dining table, toilet, tv, laptop, mouse, remote, keyboard, cell phone, microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, teddy bear, hair drier, toothbrush
</details>

---

## 输出话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/detections` | Detection2DArray | 检测框、类别、置信度 |
| `/centers` | PoseArray | 中心点坐标（像素）+ 深度值（米）<br>position.x/y: 像素坐标<br>position.z: 深度（米） |
| `/detections_visualization` | Image | 带标注的可视化图像（包含深度信息） |

---

## 故障排查

**问题1：NumPy 版本错误**
```bash
pip3 install "numpy<2" "opencv-python<4.9" -i https://mirrors.aliyun.com/pypi/simple/
```

**问题2：没有检测到物体**
- 检查 `target_list` 是否包含你要检测的物体
- 降低 `conf` 阈值（如 0.2）
- 确认图像话题正确：`ros2 topic list | grep image`

**问题3：深度值为 0**
- 确认深度话题正确：`ros2 topic echo /stereo/depth`
- 检查相机是否正常输出深度图
- 深度相机可能在某些区域无法测量（如太近或太远）

**问题4：模型加载失败**
```bash
ls -lh ~/ros2_ws/src/yolo_center_detector/resource/models/yolo26s.pt
```

---

## 首次安装依赖

```bash
# ROS2 依赖
sudo apt update
sudo apt install -y ros-humble-cv-bridge ros-humble-vision-msgs

# Python 依赖
pip3 install ultralytics opencv-python -i https://mirrors.aliyun.com/pypi/simple/
```