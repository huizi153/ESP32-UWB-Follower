# Go2 UWB Follower

UWB 引导的宇树 Go2 EDU 仿狗跟随控制器。从 ESP32 串口读取 UWB 测距测角数据（格式：`距离cm,方位角度`），通过 ROS2 话题 `/api/sport/request` 控制机器狗像狗一样跟随信号源，保持约 1m 距离。

## 文件说明

| 文件 | 作用 |
|------|------|
| `go2_uwb_follower.cpp` | ROS2 控制节点（C++） |
| `package.xml` | ROS2 包清单 |
| `CMakeLists.txt` | 编译配置 |


## 依赖

- ROS2 Humble
- `unitree_api` 消息包（随 Go2 教程安装）
- CycloneDDS（绑定连狗的网卡）

## 编译

```bash
# 放到工作空间
mkdir -p ~/unitree_go2_ws/src/go2_uwb_follower
cp go2_uwb_follower.cpp CMakeLists.txt package.xml ~/unitree_go2_ws/src/go2_uwb_follower/

# 编译
cd ~/unitree_go2_ws
colcon build --packages-select go2_uwb_follower
source install/setup.bash
```

## 运行

```bash
# 1. ROS2 + CycloneDDS + 网卡绑定
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enx00e04c5a4608" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

# 2. 激活工作空间
source ~/unitree_go2_ws/install/setup.bash

# 3. 启动跟随（串口默认 /dev/ttyUSB0，可通过参数覆盖）
sudo chmod 666 /dev/ttyACM0
ros2 run go2_uwb_follower go2_uwb_follower --ros-args -p serial_port:=/dev/ttyACM0
```

## 控制原理

```
ESP32 串口 (~15Hz)
  │  "距离cm,方位角度"  (如 "15,25" = 15cm, 左偏25°)
  ▼
帧累积器 (200ms 周期)
  │  尖峰过滤 → 多帧平均 → 低通滤波
  ▼
PID 控制器
  │  距离: P + I + D    角度: 纯 P
  │  死区: 距离 ±15cm / 角度 ±8°
  ▼
/api/sport/request
  │  unitree_api/msg/Request
  │  api_id=1008, param={"x":vx,"y":0,"z":vyaw}
  ▼
Go2 真机
```

**指令限频**: 每 200ms 发布一条 Move 指令（5Hz），防止高频指令导致抽搐。

**信号丢失**: 3 秒超时 → 原地旋转搜索 → 8 秒搜索超时 → 停止等待信号恢复。

## 可调参数

所有参数在 `go2_uwb_follower.cpp` 顶部 `namespace cfg` 中，改数值重编译即可：

- `TARGET_DISTANCE_CM`: 理想跟随距离（默认 100cm）
- `DIST_DEADZONE_CM`: 距离死区（默认 15cm）
- `ANGLE_DEADZONE_DEG`: 角度死区（默认 8°）
- `KP_DIST / KI_DIST / KD_DIST`: 距离 PID 增益
- `KP_ANGLE`: 角度 P 增益
- `CMD_INTERVAL`: Move 指令间隔（默认 200ms）
- `FILTER_ALPHA`: 低通滤波系数（默认 0.15，越小越平滑）
