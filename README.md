# Go2 UWB Follower —— UWB 引导的宇树 Go2 EDU 仿狗跟随控制器

基于 UWB 测距测角数据, 通过 ROS2 话题控制宇树 Go2 EDU 像狗一样跟随信号源, 保持约 1m 距离。

## 运行环境

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2 发行版**: Humble
- **通信中间件**: CycloneDDS
- **机器人**: 宇树 Go2 EDU

## 前置条件 (必读)

本项目是**自动跟随控制层**。在此之前必须先打通电脑与机器狗的基础通信链路。
这一部分详见:

> **Go2 开发教程 —— 从环境搭建到键盘遥控**
> https://ztl3106742440-hub.github.io/go2-tutorial/

照着教程做完以下步骤, 确认键盘能正常控制机器狗后, 再继续本项目:

1. 安装 ROS2 Humble + unitree_api 消息包
2. 配置 CycloneDDS, 绑定连接 Go2 的网卡
3. 确认 `ros2 run go2_teleop_ctrl_keyboard go2_teleop_ctrl_keyboard` 可以遥控机器狗行走

**若键盘控制还没跑通, 先回到上方教程补齐环境, 不要跳过。**

## 项目简介

从 ESP32 串口读取 UWB 测距测角数据 (格式: `距离cm,方位角度`),
通过 ROS2 话题 `/api/sport/request` 控制机器狗像狗一样跟随信号源。

**注意**: 本节点和键盘控制节点不能同时运行——两者都往 `/api/sport/request` 发指令, 同时跑会导致狗抽搐。

## 文件说明

| 文件 | 作用 |
|------|------|
| `go2_uwb_follower.cpp` | ROS2 控制节点 (C++) |
| `package.xml` | ROS2 包清单 |
| `CMakeLists.txt` | 编译配置 |
| `README.md` | 本说明 |

## 编译

```bash
# 1. 将本目录下四个文件放入工作空间
mkdir -p ~/unitree_go2_ws/src/go2_uwb_follower
cp go2_uwb_follower.cpp CMakeLists.txt package.xml ~/unitree_go2_ws/src/go2_uwb_follower/

# 2. 编译
cd ~/unitree_go2_ws
colcon build --packages-select go2_uwb_follower
source install/setup.bash
```

## 运行

```bash
# 1. ROS2 环境 + CycloneDDS + 网卡绑定 (与键盘控制完全相同)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enx00e04c5a4608" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

# 2. 激活工作空间
source ~/unitree_go2_ws/install/setup.bash

# 3. ESP32 串口权限
sudo chmod 666 /dev/ttyACM0

# 4. 启动跟随
ros2 run go2_uwb_follower go2_uwb_follower --ros-args -p serial_port:=/dev/ttyACM0
```

**注意**: 网卡名 `enx00e04c5a4608` 和串口号 `/dev/ttyACM0` 需根据实际环境修改。

## 控制原理

```
ESP32 串口 (~15Hz)
  |  "距离cm,方位角度"  如 "15,25" = 15cm, 左偏 25度
  v
帧累积器 (200ms 周期)   尖峰过滤 -> 多帧平均 -> 低通滤波 (三层抗抖)
  v
PID 控制器              距离 P+I+D / 角度纯 P, 死区抑制微振
  v
/api/sport/request      unitree_api/msg/Request, api_id=1008
  v
Go2 真机                接收 Move(vx, vyaw) 指令
```

- **指令限频**: 每 200ms 发布一条 Move (5Hz), UWB 数据 ~15Hz 输入但指令低频输出, 防止狗抽搐
- **信号丢失**: 短时丢包忽略 -> 3 秒超时原地旋转搜索 -> 8 秒搜索超时停止等待恢复

## 可调参数

所有参数在 `go2_uwb_follower.cpp` 顶部 `namespace cfg` 中, 改数值重编译即可:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `TARGET_DISTANCE_CM` | 100 | 理想跟随距离 (cm) |
| `DIST_DEADZONE_CM` | 15 | 距离死区 (cm) |
| `ANGLE_DEADZONE_DEG` | 8 | 角度死区 (度) |
| `KP_DIST` | 0.006 | 距离比例增益 |
| `KI_DIST` | 0.0003 | 距离积分增益 |
| `KD_DIST` | 0.002 | 距离微分增益 |
| `KP_ANGLE` | 0.025 | 角度比例增益 |
| `CMD_INTERVAL` | 200ms | Move 指令间隔 |
| `FILTER_ALPHA` | 0.15 | 低通滤波系数 (越小越平滑) |
| `SIGNAL_TIMEOUT_SEC` | 3.0 | 信号丢失超时 (秒) |
| `SEARCH_DURATION_SEC` | 8.0 | 搜索持续时长 (秒) |
