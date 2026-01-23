## 基于softhand不能在ros2和ubuntu22.04及以上版本使用的原因，现在有三个解决方法

# 1 使用docker部署环境，然后在用ros1和ros2桥接

# 2 自己移植ros1 到 ros2,工程量极大

# 3 使用api控制，自己写ros2的包来进行控制
    访问串口的必要条件 将本机用户加入dialout组
    sudo usermod -aG dialout $USER

## 1）找到指定设备
### 直接扫描 `/dev/ttyUSB*` 并连接第一个找到的设备：
ros2 run qbsofthand_control qbsofthand_control_node

或者：
### 指定串口和设备 ID：
ros2 run qbsofthand_control qbsofthand_control_node --ros-args -p serial_port:=/dev/ttyUSB0 -p device_id:=1

## 2）控制闭合
使用服务service：`/qbsofthand_control_node/set_closure`

### A：按持续时间控制：`duration_sec > 0`，节点会在指定时间内插值到目标闭合度（此时速度由 `duration_sec` 决定，`speed_ratio` 不参与）
ros2 service call /qbsofthand_control_node/set_closure qbsofthand_control/srv/SetClosure "{closure: 0.7, duration_sec: 2.0, speed_ratio: 1.0}"

### B：按速度比例控制：`duration_sec == 0`，节点使用步进方式逐步逼近目标；速度由 `speed_ratio` 控制（结合参数 `max_step_per_tick`）。
ros2 service call /qbsofthand_control_node/set_closure qbsofthand_control/srv/SetClosure "{closure: 0.7, duration_sec: 0.0, speed_ratio: 0.3}"



