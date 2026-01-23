# UR5机械臂与QBSoftHand集成项目

本项目旨在将 UR5 机械臂、QBSoftHand 和 OKD 相机包集成到 ROS 2 环境中，实现多设备协同工作。

## 项目简介
由于 QBSoftHand 当前没有可用的 ROS 2 环境包，本项目通过从源代码编译 API 的方式，成功实现了基础的控制功能。未来计划进一步扩展功能，以满足更多应用场景的需求。

## 已实现功能
- QBSoftHand 的基础控制功能。
- UR5 机械臂与 QBSoftHand 的集成。
- OKD 相机包的集成。

## 待实现目标
- 实现 QBSoftHand 的速度、加速度和关节位姿获取功能。
- 将 QBSoftHand 的模型导入 RViz 进行可视化。
- 优化多设备协同控制的性能。

## 使用说明
### 环境配置
1. 确保已安装 ROS 2（推荐使用 Humble 发行版）。
2. 配置工作空间：
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

### 编译项目
在工作空间根目录下运行以下命令：
```bash
colcon build
```

### 运行说明
有关运行示例的详细说明，请参考项目目录下的以下文件：
- `01 连接ur5检查.md`
- `02 相机连接测试.md`
- `03 softhand控制.md`

## 贡献
欢迎对本项目提出建议或贡献代码。请通过提交 Issue 或 Pull Request 的方式与我们联系。

## 参考资料
- [ROS 2 官方文档](https://docs.ros.org/en/)
- [UR5 机械臂 ROS 2 驱动](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [QBSoftHand API 文档](https://qbrobotics.com/)