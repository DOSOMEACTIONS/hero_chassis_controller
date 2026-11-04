# hero_chassis_controller

麦克纳姆轮底盘控制器 —— 基于 ros_control 框架，用于控制 Gazebo 中的 RoboMaster 英雄机器人底盘。

## Overview

本包实现一个 `controller_interface::Controller<EffortJointInterface>` 插件，支持：

- **逆运动学**：订阅 `/cmd_vel`（`geometry_msgs/Twist`），计算机械纳姆轮四个轮子的目标转速
- **PID 速度闭环**：使用 `control_toolbox::Pid`，四个轮子各自独立的 PID 参数
- **正运动学 + 里程计**：由实际轮速推算车体速度，积分得到位姿，发布 `nav_msgs/Odometry` 到 `/odom`
- **TF 广播**：发布 `odom` → `base_link` 坐标变换
- **世界坐标系速度模式**：`/cmd_vel` 可解释为世界系（odom/map）速度，经 `tf` 变换到
  底盘坐标系后执行；由 `frame_mode` 参数切换（`base` / `world`）
- **加速度限制**：可配置线/角加速度上限，避免阶跃指令导致轮子打滑
- **麦轮全向移动**：支持 `linear.y` 横移（`base` 模式下左右平移，车头朝向不变）
- **可视化**：发布 `/drive/wheel_speed` 与 `/drive/wheel_target`，便于用 `rqt_plot` 观察跟踪效果

**Keywords:** ros_control, mecanum wheel, robomaster, pid, odometry

### License

The source code is released under a [MIT license](LICENSE).

**Author: heifeiyang<br />
Affiliation: Guangdong University of Technology, RoboMaster Team<br />
Maintainer: heifeiyang**

## Installation

### Building from Source

使用 catkin_tools 构建（不是 catkin_make）：

```bash
cd ~/rm_ws
catkin build hero_chassis_controller
source devel/setup.bash
```

依赖：

```bash
sudo apt install ros-noetic-control-toolbox ros-noetic-gazebo-ros-control
```

### Static code analysis

本包启用 [roslint](http://wiki.ros.org/roslint) 代码风格检查：

```bash
catkin build hero_chassis_controller --make-args roslint_hero_chassis_controller
```

代码风格由 [`.clang-format`](.clang-format) 定义（ROS C++ Style：Allman 大括号，80 列）。

## Usage

### 启动仿真

```bash
roslaunch hero_chassis_controller drive_sim.launch
```

该 launch 会依次：启动 Gazebo 与英雄模型 → 加载控制器参数 → spawn `controller/drive_controller`。

### 键盘控制

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

按住 `i` 前进、`j`/`l` 转向。

### 观察 PID 跟踪

```bash
rqt_plot /drive/wheel_speed/data[0] /drive/wheel_target/data[0]
```

### 在线调 PID 参数

```bash
rqt_reconfigure
```

左侧四个节点分别对应四个轮子的 PID（`<关节名>_pid`）。

## Config files

- **controllers.yaml** — 控制器参数

| 参数 | 说明 |
| --- | --- |
| `joints` | 四个轮子的关节名（顺序必须与逆运动学一致） |
| `wheel_radius` | 轮半径（m），默认 0.07625 |
| `k_kin` | (轮距 + 轮基) / 2（m），默认 0.4 |
| `<关节名>_pid/p,i,d,i_clamp` | 每个轮子独立的 PID 参数 |
| `frame_mode` | `base`（默认，`/cmd_vel` 为底盘系速度）/ `world`（世界系速度，经 tf 变换） |
| `world_frame` | 世界坐标系名，默认 `odom`（也可用 `map`） |
| `max_accel_linear` | 线速度最大变化率 m/s²，`0` = 不限制 |
| `max_accel_angular` | 角速度最大变化率 rad/s²，`0` = 不限制 |

## Launch files

- **drive_sim.launch** — 一键启动（Gazebo + 英雄模型 + 控制器）
- **drive.launch** — 只加载控制器参数并 spawn（需已有 Gazebo）

## Nodes

本包**不提供独立节点**。控制器以 pluginlib 插件形式被 `controller_manager` 加载进
`gzserver` 进程，因此 `rosnode list` 中不会出现独立节点，其订阅/发布均署名 `/gazebo`。

### 接口

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令（车身坐标系） |
| 发布 | `/odom` | `nav_msgs/Odometry` | 里程计（pose + twist） |
| 发布 | `/tf` | `tf2_msgs/TFMessage` | odom → base_link |
| 发布 | `/drive/wheel_speed` | `std_msgs/Float64MultiArray` | 四个轮子的实际转速 [fl, fr, rl, rr] |
| 发布 | `/drive/wheel_target` | `std_msgs/Float64MultiArray` | 四个轮子的目标转速 [fl, fr, rl, rr] |

## 实现说明

### 逆运动学

```
w_fl = (vx - vy - k·wz) / r
w_fr = (vx + vy + k·wz) / r
w_rl = (vx + vy - k·wz) / r
w_rr = (vx - vy + k·wz) / r
```

其中 `k = (轮距 + 轮基) / 2`，`r` 为轮半径。

### 世界坐标系速度模式

`frame_mode: world` 时，`/cmd_vel` 被解释为世界坐标系下的速度。控制器在每个控制周期用
`tf::TransformListener::transformVector()` 把它变换到 `base_link` 系后再送入逆运动学：

```
世界系速度向量 --tf--> 底盘系速度向量 --> 逆运动学
```

实测（同一指令 `vx=0.5`，车头已转约 140°）：

| 模式 | 位移方向 | 含义 |
| --- | --- | --- |
| `base` | 210°（车头方向） | 车沿自身车头前进 |
| `world` | 356°（≈世界 +x） | 车沿世界 +x 平移，车头不变 |

### 加速度限制

`max_accel_linear` / `max_accel_angular` 大于 0 时，目标速度每周期最多变化
`max_accel × dt`，把阶跃指令变成斜坡：

```
目标速度 ──速率限幅──> 斜坡目标 ──PID──> 力矩
```

限制作用于**目标值**而非 PID 输出——若限制输出，会连同 PID 的正常调节一起被削掉。

实测（`vx=0.5`，目标轮速 6.56 rad/s）：

| `max_accel_linear` | 目标上升时间 | 轮速实际上升时间 |
| --- | --- | --- |
| `0`（不限制） | 0.000 s（阶跃） | 0.141 s |
| `1.0` m/s² | 0.402 s（斜坡） | 0.462 s |

### 控制频率

`update()` 由 ros_control 以 1 kHz 调用。可视化话题降频至 50 Hz，避免压垮 `rqt_plot`。

### 坐标系约定

遵循 REP-103：x 前、y 左、z 上；绕 z 逆时针为正。
`/cmd_vel` 的 `linear.x` 与 `angular.z` 分别对应前进速度与转向角速度。

## Bugs & Feature Requests

请通过 GitHub Issue 反馈。
