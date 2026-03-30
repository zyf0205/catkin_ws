# PX4 + MAVROS + MID360 + FAST-LIO2 + EGO-Planner 仿真项目

基于 ROS1 的无人机自主飞行仿真系统，集成固态激光雷达里程计、轨迹规划与 PX4 飞控，支持 Gazebo 仿真与真实硬件部署。

---

## 目录

- [系统概述](#系统概述)
- [整体架构](#整体架构)
- [数据流向](#数据流向)
- [模块说明](#模块说明)
- [关键配置参数](#关键配置参数)
- [目录结构](#目录结构)
- [环境依赖](#环境依赖)
- [快速启动](#快速启动)
- [坐标系说明](#坐标系说明)
- [常见问题](#常见问题)

---

## 系统概述

本项目构建了一套完整的自主无人机系统，各核心模块职责如下：

| 模块 | 功能 |
|------|------|
| **PX4 SITL** | 飞控固件，提供底层电机控制与 EKF2 状态估计 |
| **MAVROS** | ROS ↔ PX4 MAVLink 通信桥接 |
| **Livox MID360** | 360° 固态激光雷达 + 内置 IMU，提供原始点云与惯性数据 |
| **FAST-LIO2** | 激光雷达-IMU 紧耦合里程计，输出高精度位姿与三维地图 |
| **EGO-Planner** | 基于 B 样条优化的实时避障轨迹规划器 |
| **drone_ctrl** | 坐标系转换 + 模式切换 + 控制指令下发（自定义包） |
| **px4_fastlio_bridge** | 将 FAST-LIO 里程计转发至 PX4 EKF2 视觉融合接口（自定义包） |
| **Mid360_imu_sim** | Gazebo 中 MID360 + IMU 仿真模型 |

---

## 整体架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                          硬件 / 仿真层                               │
│      PX4 SITL (Gazebo)          Livox MID360 (Gazebo 仿真插件)      │
└──────────┬──────────────────────────────┬───────────────────────────┘
           │ MAVLink                   │ UDP 点云 + IMU
           ▼                           ▼
┌──────────────────┐        ┌──────────────────────────┐
│     MAVROS       │        │   livox_ros_driver2      │
│  (ROS 通信桥)    │        │   (Livox 官方 ROS 驱动)  │
└────────┬─────────┘        └────────────┬─────────────┘
         │                               │
         │ /mavros/local_position/pose   │ /scan (CustomMsg)
         │ /mavros/state                 │ /livox/imu (IMU)
         │                               ▼
         │                  ┌────────────────────────────┐
         │                  │       FAST-LIO2            │
         │                  │  (激光雷达-IMU 里程计)     │
         │                  │  · ikd-Tree 增量建图        │
         │                  │  · 迭代卡尔曼滤波           │
         │                  └──┬─────────┬──────────────┘
         │                     │         │
         │         /Odometry   │         │ /cloud_registered
         │         (位姿+速度) │         │ (已配准点云)
         │                     │         │
         │        ┌────────────┘         │
         │        │                      │
         │        ▼                      ▼
         │  ┌─────────────────┐  ┌───────────────────────────┐
         │  │px4_fastlio_     │  │      EGO-Planner          │
         │  │bridge           │  │   (轨迹规划与避障)        │
         │  │odom_to_vision.py│  │  · 局部代价地图           │
         │  └────────┬────────┘  │  · B 样条轨迹优化         │
         │           │           └──────────────┬────────────┘
         │           │ /mavros/                  │
         │           │ vision_pose/pose          │ /planning/pos_cmd
         │           │ (视觉里程计融合)          │ (位置+速度+加速度)
         │           ▼                           ▼
         │      PX4 EKF2               ┌─────────────────────────┐
         │      (外部位姿融合)         │    drone_ctrl_node      │
         │                             │  · 坐标系偏移标定       │
         └────────────────────────────►│  · 手动/自动模式切换    │
                                       │  · 键盘遥控接口         │
                                       └──────────┬──────────────┘
                                                  │ /mavros/setpoint_raw/local
                                                  ▼
                                          PX4 OFFBOARD 控制器
                                          (SO3 姿态 → 电机 PWM)
```

---

## 数据流向

### ROS Topic 完整流向表

| 阶段 | 发布节点 | Topic | 消息类型 | 订阅节点 | 频率 |
|------|----------|-------|----------|----------|------|
| 传感 | livox_ros_driver2 / Gazebo 插件 | `/scan` | `livox_ros_driver2/CustomMsg` | FAST-LIO2 | 10 Hz |
| 传感 | livox_ros_driver2 / Gazebo 插件 | `/livox/imu` | `sensor_msgs/Imu` | FAST-LIO2 | ~200 Hz |
| 里程计 | FAST-LIO2 | `/Odometry` | `nav_msgs/Odometry` | EGO-Planner, drone_ctrl, bridge | 10+ Hz |
| 建图 | FAST-LIO2 | `/cloud_registered` | `sensor_msgs/PointCloud2` | EGO-Planner | 10+ Hz |
| 轨迹历史 | FAST-LIO2 | `/path` | `nav_msgs/Path` | RViz 可视化 | 10+ Hz |
| 视觉融合 | px4_fastlio_bridge | `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | PX4 EKF2 | 10+ Hz |
| PX4 状态 | MAVROS | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | drone_ctrl | 50 Hz |
| PX4 状态 | MAVROS | `/mavros/state` | `mavros_msgs/State` | drone_ctrl | 1 Hz |
| 航点目标 | waypoint_generator | `/waypoint_generator/waypoints` | `nav_msgs/Path` | EGO-Planner | 事件触发 |
| 规划指令 | EGO-Planner (traj_server) | `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | drone_ctrl | 50 Hz |
| 控制指令 | drone_ctrl | `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | PX4 (OFFBOARD) | 50 Hz |

### 关键数据流简述

```
MID360 点云 + IMU
        │
        ▼
  FAST-LIO2 融合
        │
    ┌───┴───────────────────┐
    │                       │
    ▼                       ▼
/Odometry             /cloud_registered
    │                       │
    ├──► bridge ──► PX4 EKF2 (位姿融合)
    │
    └──► EGO-Planner ◄──── /cloud_registered (障碍物)
              │
              ▼
    /planning/pos_cmd
              │
              ▼
    drone_ctrl（坐标系转换 + 模式选择）
              │
              ▼
    /mavros/setpoint_raw/local
              │
              ▼
        PX4 飞控执行
```

---

## 模块说明

### 1. livox_ros_driver2 — MID360 驱动

**路径：** `src/livox_ros_driver2/`

官方 Livox ROS 驱动，负责与 MID360 设备通信并发布点云和 IMU 数据。

**关键启动文件：**
- `launch_ROS1/msg_MID360.launch` — 纯数据发布（无 RViz）
- `launch_ROS1/rviz_MID360.launch` — 数据发布 + RViz 可视化

**网络配置（`config/MID360_config.json`，注释仅作说明，实际 JSON 不含注释）：**
```json
{
  "lidar_configs": [{
    "ip": "192.168.1.12",
    "pcl_data_type": 1
  }],
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "192.168.1.5",
      "point_data_port": 56301,
      "imu_data_port": 56401
    }
  }
}
```
> `ip` 为 MID360 设备 IP；`point_data_ip/port` 为主机接收点云的地址与端口；`imu_data_ip/port` 为主机接收 IMU 的地址与端口。

---

### 2. Mid360_imu_sim — Gazebo 仿真模型

**路径：** `src/Mid360_imu_sim/`

在 Gazebo 中仿真 MID360 激光雷达和 IMU，输出与真实驱动相同的 ROS Topic。

**关键启动文件：**
- `launch/mid360_IMU_platform.launch` — 启动 Gazebo + 无人机 + 传感器模型
- `launch/livox_simulation.launch` — 含 RViz 的完整仿真场景

**重要参数：**
```xml
<arg name="use_sim_time" default="true"/>  <!-- 使用 Gazebo 仿真时钟 -->
```

---

### 3. FAST_LIO — 激光雷达里程计与建图

**路径：** `src/FAST_LIO/`

基于迭代扩展卡尔曼滤波（IEKF）和 ikd-Tree 的激光雷达-IMU 紧耦合里程计，提供高精度位姿估计和稠密三维地图。

**关键启动文件：**
- `launch/mapping_mid360.launch` — 针对 MID360 的建图配置

**MID360 配置文件（`config/mid360.yaml`）：**
```yaml
common:
    lid_topic:  "/scan"         # 激光雷达输入 Topic
    imu_topic:  "/livox/imu"   # IMU 输入 Topic
    time_sync_en: false

preprocess:
    lidar_type: 1       # 1 = Livox 系列（无需特征提取）
    scan_line: 4        # MID360 有效扫描线数
    blind: 0.5          # 盲区半径（米），忽略过近点云

mapping:
    acc_cov: 0.05       # 加速度计噪声协方差
    gyr_cov: 0.05       # 陀螺仪噪声协方差
    b_acc_cov: 0.00001  # 加速度计零偏协方差
    b_gyr_cov: 0.00001  # 陀螺仪零偏协方差
    fov_degree: 360     # 全方位水平视场角
    det_range: 100.0    # 最大探测距离（米）
    extrinsic_est_en: false       # 关闭在线外参标定
    extrinsic_T: [0, 0, 0]       # IMU 到 LiDAR 平移偏移（米）
    extrinsic_R: [1,0,0, 0,1,0, 0,0,1]  # 旋转偏移（单位矩阵）
```

---

### 4. ego_planner — 轨迹规划器

**路径：** `src/ego_planner/`

基于梯度的 B 样条轨迹优化规划器，实时生成无碰撞轨迹。

**关键启动文件：**
- `src/planner/plan_manage/launch/simple_run.launch` — 真实硬件 / PX4 SITL 场景
- `src/planner/plan_manage/launch/run_in_sim.launch` — 纯仿真场景（含虚拟深度相机）

**主要规划参数（`simple_run.launch`）：**
```xml
<arg name="odom_topic"         value="/Odometry" />         <!-- 里程计输入 -->
<arg name="cloud_topic"        value="/cloud_registered" /> <!-- 障碍物点云输入 -->
<arg name="map_size_x_"        value="40.0"/>               <!-- 局部地图 X（米）-->
<arg name="map_size_y_"        value="40.0"/>               <!-- 局部地图 Y（米）-->
<arg name="map_size_z_"        value="10.0"/>               <!-- 局部地图 Z（米）-->
<arg name="max_vel"            value="1.2" />               <!-- 最大速度（m/s）-->
<arg name="max_acc"            value="2.0" />               <!-- 最大加速度（m/s²）-->
<arg name="planning_horizon"   value="7.5" />               <!-- 规划前视距离（米）-->
<arg name="flight_type"        value="1" />                 <!-- 1=RViz 2D Goal，2=预设航点 -->
```

**SO3 控制增益（`src/uav_simulator/so3_control/config/gains.yaml`）：**
```yaml
gains:
  pos:
    x: 5.0   # X 位置增益
    y: 5.0   # Y 位置增益
    z: 15.0  # Z 位置增益（高度控制更强）
  vel:
    x: 5.0   # X 速度阻尼
    y: 5.0
    z: 5.0
  rot:
    x: 3.5   # 滚转/俯仰增益
    y: 3.5
    z: 1.0   # 偏航增益（较弱）
  ang:
    x: 0.4   # 角速率阻尼
    y: 0.4
    z: 0.1
```

---

### 5. px4_fastlio_bridge — 里程计桥接节点

**路径：** `src/px4_fastlio_bridge/scripts/odom_to_vision.py`

将 FAST-LIO2 输出的 `nav_msgs/Odometry` 转换为 PX4 EKF2 可接收的视觉里程计格式，实现外部位姿融合。

**工作原理：**
```python
# 订阅 FAST-LIO 里程计
/Odometry  (nav_msgs/Odometry)
        │
        ▼  时间戳替换为 rospy.Time.now()（PX4 时间对齐）
        │  坐标直接透传（位置 + 四元数姿态）
        ▼
/mavros/vision_pose/pose  (geometry_msgs/PoseStamped)
        │
        ▼  → PX4 EKF2 外部视觉融合
```

> **注意：** 使用 `rospy.Time.now()` 而非原始时间戳，确保与 PX4 EKF2 的时间一致性。

---

### 6. drone_ctrl — 无人机控制节点

**路径：** `src/drone_ctrl/scripts/drone_ctrl_node.py`

核心控制接口，负责坐标系转换、飞行模式切换和控制指令下发。

**订阅 Topic：**
- `/planning/pos_cmd` — EGO-Planner 轨迹指令
- `/Odometry` — FAST-LIO2 里程计（用于坐标系标定）
- `/mavros/local_position/pose` — PX4 当前位姿反馈

**发布 Topic / 服务调用：**
- `/mavros/setpoint_raw/local` — OFFBOARD 位置速度控制指令
- `/mavros/set_mode` — 切换飞行模式（OFFBOARD）
- `/mavros/cmd/arming` — 解锁/上锁

**坐标系转换：**
```
camera_init (FAST-LIO) ──► local_NED (PX4)
offset = mav_pose - lio_pose  （标定时计算一次）
转换公式：mav_target = lio_target + offset
```

**键盘控制说明：**

| 按键 | 功能 |
|------|------|
| `W / S` | 前进 / 后退（±0.2 m） |
| `A / D` | 左移 / 右移（±0.2 m） |
| `↑ / ↓` | 上升 / 下降（±0.1 m） |
| `← / →` | 偏航左转 / 右转（±0.15 rad） |
| `E` | 启用 EGO-Planner 自动规划模式 |
| `M` | 切换回手动控制模式 |
| `C` | 重新标定坐标系偏移 |
| `Q` | 退出节点 |

---

## 关键配置参数

### FAST-LIO2 参数汇总

| 参数 | 文件 | 值 | 说明 |
|------|------|----|------|
| `lidar_type` | `mid360.yaml` | 1 | Livox 系列（无特征提取） |
| `scan_line` | `mid360.yaml` | 4 | 有效扫描线数 |
| `blind` | `mid360.yaml` | 0.5 m | 盲区半径 |
| `det_range` | `mid360.yaml` | 100.0 m | 最大探测距离 |
| `fov_degree` | `mid360.yaml` | 360° | 全向水平视场 |
| `extrinsic_est_en` | `mid360.yaml` | false | 关闭在线外参标定 |
| `filter_size_surf` | `mapping_mid360.launch` | 0.5 | 体素滤波分辨率（米） |
| `filter_size_map` | `mapping_mid360.launch` | 0.5 | 地图体素分辨率（米） |

### EGO-Planner 参数汇总

| 参数 | 文件 | 值 | 说明 |
|------|------|----|------|
| `max_vel` | `simple_run.launch` | 1.2 m/s | 最大飞行速度 |
| `max_acc` | `simple_run.launch` | 2.0 m/s² | 最大加速度 |
| `planning_horizon` | `simple_run.launch` | 7.5 m | 前视规划距离 |
| `map_size_x/y/z` | `simple_run.launch` | 40/40/10 m | 局部地图尺寸 |
| `flight_type` | `simple_run.launch` | 1 | 1=RViz 目标点，2=预设航点 |

### MID360 网络参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 设备 IP | `192.168.1.12` | MID360 的 IP 地址 |
| 主机 IP | `192.168.1.5` | 运行驱动的电脑 IP |
| 点云端口（设备侦听 → 主机接收）| 56300 → 56301 | MID360 在 56300 发送，主机在 56301 接收 |
| IMU 端口（设备侦听 → 主机接收） | 56400 → 56401 | MID360 在 56400 发送，主机在 56401 接收 |

---

## 目录结构

```
catkin_ws/
├── scripts/
│   └── fastlio_ego_sim.sh              # 全系统一键启动脚本
└── src/
    ├── CMakeLists.txt                  # Catkin 工作空间顶层 CMake
    │
    ├── livox_ros_driver2/              # Livox 官方 ROS 驱动
    │   ├── config/
    │   │   └── MID360_config.json      # 设备 IP / 端口 / 外参配置
    │   └── launch_ROS1/
    │       ├── msg_MID360.launch       # 纯数据发布
    │       └── rviz_MID360.launch      # 数据发布 + RViz
    │
    ├── FAST_LIO/                       # FAST-LIO2 里程计与建图
    │   ├── config/
    │   │   └── mid360.yaml             # LiDAR-IMU 融合参数
    │   ├── launch/
    │   │   └── mapping_mid360.launch   # MID360 建图启动文件
    │   └── src/
    │       └── laserMapping.cpp        # 核心算法实现
    │
    ├── Mid360_imu_sim/                 # Gazebo MID360 仿真
    │   ├── launch/
    │   │   ├── mid360_IMU_platform.launch   # 基础 Gazebo 仿真
    │   │   └── livox_simulation.launch      # 含 RViz 的完整仿真
    │   ├── urdf/
    │   │   └── mid360_IMU_platform.xacro    # 传感器 + 无人机 URDF
    │   └── worlds/
    │       └── standardrobots_factory.world # Gazebo 仿真场景
    │
    ├── ego_planner/                    # EGO-Planner 轨迹规划器
    │   └── src/
    │       ├── planner/
    │       │   ├── plan_manage/launch/
    │       │   │   ├── simple_run.launch       # SITL / 实机启动
    │       │   │   ├── run_in_sim.launch       # 纯仿真启动
    │       │   │   └── advanced_param.xml      # 规划参数模板
    │       │   ├── plan_env/                   # 局部代价地图
    │       │   ├── path_searching/             # 路径搜索（RRT*）
    │       │   ├── bspline_opt/                # B 样条轨迹优化
    │       │   └── traj_utils/                 # 轨迹工具库
    │       └── uav_simulator/
    │           ├── so3_control/config/
    │           │   └── gains.yaml              # 姿态控制增益
    │           └── local_sensing/params/
    │               └── camera.yaml             # 虚拟相机内参
    │
    ├── drone_ctrl/                     # 无人机控制节点（自定义）
    │   └── scripts/
    │       └── drone_ctrl_node.py      # 键盘控制 + 自动模式 + 坐标转换
    │
    └── px4_fastlio_bridge/             # LIO → PX4 里程计桥（自定义）
        └── scripts/
            └── odom_to_vision.py       # Odometry → vision_pose 转换
```

---

## 环境依赖

- **操作系统：** Ubuntu 20.04
- **ROS：** ROS Noetic
- **PX4：** v1.13 或更高版本（PX4 Autopilot 固件，需支持 SITL 与 EKF2 外部视觉融合）
- **Gazebo：** 11.x
- **Python：** 3.8+

**ROS 依赖包：**
```
mavros  mavros_msgs  geometry_msgs  nav_msgs  sensor_msgs
pcl_ros  tf  roscpp  rospy
quadrotor_msgs  livox_ros_driver2  livox_laser_simulation
```

---

## 快速启动

### 1. 编译工作空间

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动完整仿真（推荐）

使用一键启动脚本，自动按顺序打开各模块终端：

```bash
cd ~/catkin_ws
bash scripts/fastlio_ego_sim.sh
```

脚本启动顺序：
1. **PX4 SITL** — `roslaunch px4 mavros_posix_sitl.launch`
2. **FAST-LIO2** — `roslaunch fast_lio mapping_mid360.launch use_sim_time:=true`
3. **桥接节点** — `rosrun px4_fastlio_bridge odom_to_vision.py`
4. **EGO-Planner** — `roslaunch ego_planner simple_run.launch`
5. **QGroundControl** — 地面站软件
6. **控制节点** — `rosrun drone_ctrl drone_ctrl_node.py`（等待所有节点就绪后手动确认）

### 3. 分步启动（调试用）

```bash
# 终端 1：PX4 SITL + MAVROS
roslaunch px4 mavros_posix_sitl.launch

# 终端 2：Gazebo MID360 仿真（若使用仿真雷达）
roslaunch Mid360_imu_sim mid360_IMU_platform.launch

# 终端 3：FAST-LIO2 建图
roslaunch fast_lio mapping_mid360.launch use_sim_time:=true

# 终端 4：里程计桥接
rosrun px4_fastlio_bridge odom_to_vision.py

# 终端 5：EGO-Planner
roslaunch ego_planner simple_run.launch

# 终端 6：无人机控制节点
rosrun drone_ctrl drone_ctrl_node.py
```

### 4. 飞行操作流程

1. 启动所有节点后，在控制节点终端等待 MAVROS 连接
2. 按 `C` 键标定坐标系偏移（首次使用必做）
3. 按 `M` 键进入手动模式，操控无人机起飞
4. 在 RViz 中使用 **2D Nav Goal** 工具点击目标位置
5. 按 `E` 键切换为 EGO-Planner 自动规划模式
6. 按 `M` 键随时切回手动控制

---

## 坐标系说明

本系统涉及两个主要坐标系，**必须通过标定正确对齐**：

| 坐标系 | 来源 | 原点 | 说明 |
|--------|------|------|------|
| `camera_init` | FAST-LIO2 | FAST-LIO2 启动时的传感器位置 | 里程计参考系 |
| `local_NED` | PX4 / MAVROS | 无人机解锁时的位置 | PX4 控制参考系 |

**偏移标定流程：**
- `drone_ctrl_node.py` 在按下 `C` 键时，计算两个坐标系的平移偏移量
- 后续所有控制指令均自动叠加该偏移
- 若 FAST-LIO2 重启或无人机重新解锁，**需重新标定**

---

## 常见问题

**Q：FAST-LIO2 频繁退出或漂移严重？**
- 检查 `mid360.yaml` 中 `extrinsic_T` 和 `extrinsic_R` 是否与实际 IMU-LiDAR 安装关系一致
- 确保 `blind` 参数不小于传感器到机架的最短距离
- 仿真时确认 `use_sim_time:=true` 已生效

**Q：PX4 无法进入 OFFBOARD 模式？**
- 确认 `/mavros/setpoint_raw/local` 以 ≥2 Hz 持续发布
- 检查 MAVROS 连接状态：`rostopic echo /mavros/state`
- 仿真中需在 QGroundControl 中先设置 OFFBOARD 模式或通过脚本自动切换

**Q：EGO-Planner 无法规划路径？**
- 确认 `/Odometry` 和 `/cloud_registered` 正常发布
- 检查目标点是否位于局部地图范围内（默认 40×40×10 m）
- 在 RViz 中查看代价地图是否正确更新

**Q：坐标系偏移导致飞行异常？**
- 在无人机静止状态下重新按 `C` 键标定
- 确保标定时 FAST-LIO2 里程计已稳定（无初始化漂移）

**Q：真实 MID360 无法连接？**
- 确认主机 IP 设置为 `192.168.1.5`，设备 IP 为 `192.168.1.12`
- 检查防火墙规则：`sudo ufw allow 56100:56500/udp`
- 参考 `src/livox_ros_driver2/README.md` 中的网络配置说明
