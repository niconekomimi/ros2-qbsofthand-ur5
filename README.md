# UR5 机械臂视觉抓取系统

集成 UR5 机械臂、OAK-D 相机、QB Soft Hand 灵巧手的 ROS2 视觉协同系统。通过手眼标定实现在相机坐标系中识别物体，并转换到机械臂坐标系进行精确抓取。

---

## ✨ 主要功能

- ✅ UR5 机械臂驱动与控制
- ✅ OAK-D 相机集成
- ✅ QB Soft Hand 灵巧手控制
- ✅ ArUco 标记手眼标定
- ✅ 坐标系变换与可视化

---

## 🚀 快速开始

### 环境配置

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build
```

灵巧手需要串口权限：
```bash
sudo usermod -aG dialout $USER
```

### 日常启动（5 个终端）

```bash
# 终端 1：相机
ros2 launch depthai_examples rgb_stereo_node.launch.py

# 终端 2：机械臂
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.211 launch_rviz:=false reverse_ip:=192.168.1.10

# 终端 3：手眼标定（**必须**）
ros2 launch easy_handeye2 publish.launch.py name:=ur5_oak_eyehand

# 终端 4：灵巧手
ros2 run qbsofthand_control qbsofthand_control_node

# 终端 5：应用程序
python3 your_app.py
```

> ⚠️ **关键**：终端 3 的标定发布命令不可跳过

---

## 📚 文档

| 文档 | 内容 |
|------|------|
| [01 连接ur5检查.md](01%20连接ur5检查.md) | UR5 启动、网络、测试 |
| [02 相机连接测试.md](02%20相机连接测试.md) | OAK-D 相机驱动与调试 |
| [03 softhand控制.md](03%20softhand控制.md) | 灵巧手控制方法 |
| [04 手眼标定.md](04%20手眼标定.md) | **关键**：完整标定流程 |

---

## 🛠️ 工具脚本

- `tools/aruco_transform_to_tf.py` - ArUco TF 转发（当识别到标记但 TF 不发布时使用）
- `tools/ur_send_small_trajectory.py` - UR5 运动测试脚本

---

## 📦 项目结构

```
ros2_ws/
├── src/
│   ├── qbsofthand_control/      # 灵巧手控制包（本项目）
│   └── ...                      # 其他第三方包
├── tools/
├── 01-04.md                     # 详细文档
└── README.md
```

---

## 🔗 参考资源

- [ROS 2 文档](https://docs.ros.org/en/)
- [UR ROS2 驱动](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)
- [depthai-ros](https://github.com/luxonis/depthai-ros)