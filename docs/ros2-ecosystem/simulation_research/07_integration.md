# 仿真栈与本仓库已有组件的对接

> 本文档将 Phase 9 仿真技术栈与本仓库已有组件（`chassis_protocol`、Phase 3 VLA 训练、Phase 6 rosbag2、Phase 7 RT 内核）进行具体对接，给出可落地的集成方案和决策矩阵。

---

## 一、chassis_protocol 仿真 Hardware Interface

### 1.1 现状与目标

当前 `chassis_protocol` 的硬件访问层（`chassis_hal.cpp`）直接依赖 RS485/TCP/UDP 传输层。在仿真中需要将该层无损替换为 Gazebo/Isaac 提供的虚拟关节驱动，同时保持上层 `controller_manager` → `ros2_control` → 控制器的接口完全一致。

```
目标架构（统一接口）：

                     ros2_control
                   controller_manager
                          │
                  SystemInterface 抽象层
                ┌─────────┴──────────┐
                │                    │
      ┌─────────▼──────┐  ┌──────────▼────────┐
      │  实机 HAL        │  │  仿真 Plugin       │
      │  ChassisSystem  │  │  GazeboSystem /   │
      │  (RS485 真实通信)│  │  IsaacSystem      │
      └─────────────────┘  └───────────────────┘
              │                       │
       真实底盘驱动器            Gazebo/Isaac Sim
      (motors + encoders)        (虚拟关节状态)
```

### 1.2 GazeboSystem 插件设计方案

```cpp
// chassis_protocol/sim/gazebo_chassis_system.hpp

#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>

namespace chassis_protocol_sim {

class GazeboChassissSystem
    : public hardware_interface::SystemInterface,
      public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate
{
public:
    // ── hardware_interface::SystemInterface ─────────────────────────
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    // ── gz::sim::System ──────────────────────────────────────────────
    void Configure(const gz::sim::Entity & entity,
                   const std::shared_ptr<const sdf::Element> & sdf,
                   gz::sim::EntityComponentManager & ecm,
                   gz::sim::EventManager & eventMgr) override;

    void PreUpdate(const gz::sim::UpdateInfo & info,
                   gz::sim::EntityComponentManager & ecm) override;

    void PostUpdate(const gz::sim::UpdateInfo & info,
                    const gz::sim::EntityComponentManager & ecm) override;

private:
    // 关节状态/命令缓存（无锁，供 RT 控制环读写）
    std::vector<double> hw_positions_;    // 从 Gazebo 读取
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    std::vector<double> hw_commands_;    // 写入 Gazebo

    gz::sim::Entity model_entity_;
    std::vector<gz::sim::Entity> joint_entities_;
};

}  // namespace chassis_protocol_sim
```

### 1.3 xacro sim_mode 条件切换

```xml
<!-- chassis_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot name="chassis_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 仿真模式参数 -->
  <xacro:arg name="sim_mode" default="false"/>
  <xacro:arg name="use_gazebo" default="false"/>

  <!-- ros2_control 配置块 -->
  <ros2_control name="ChassisSystem" type="system">
    <hardware>
      <xacro:if value="$(arg use_gazebo)">
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </xacro:if>
      <xacro:unless value="$(arg use_gazebo)">
        <xacro:if value="$(arg sim_mode)">
          <!-- Isaac Sim 使用 mock_components -->
          <plugin>mock_components/GenericSystem</plugin>
          <param name="fake_sensor_commands">false</param>
        </xacro:if>
        <xacro:unless value="$(arg sim_mode)">
          <!-- 实机：使用 chassis_protocol 实现 -->
          <plugin>chassis_protocol/ChassisSystem</plugin>
          <param name="transport_type">rs485</param>
          <param name="device">/dev/ttyUSB0</param>
          <param name="baudrate">1000000</param>
        </xacro:unless>
      </xacro:unless>
    </hardware>
    <!-- 关节接口定义（实机/仿真共用） -->
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
```

### 1.4 ChassisHal Mock（纯仿真模式）

```cpp
// chassis/sim/chassis_hal_mock.cpp
// 用于 Isaac Sim 中不通过 gz_ros2_control 的轻量仿真

#include "chassis/chassis_hal.hpp"

namespace chassis_protocol {

class ChassisHalMock : public ChassisHal {
public:
    explicit ChassisHalMock() = default;

    bool init(const ChassisConfig & cfg) override {
        wheel_base_ = cfg.wheel_base;
        wheel_radius_ = cfg.wheel_radius;
        return true;  // 无实际硬件初始化
    }

    void write(const ChassisCommand & cmd) override {
        // 将速度指令转发到 Isaac Sim ROS 2 话题
        // 实际由 OmniGraph → PhysX Articulation Drive 执行
        last_cmd_ = cmd;
    }

    ChassisState read() override {
        // 从 /joint_states 话题读取（由 ros2_control → Isaac Bridge 提供）
        return cached_state_;
    }

    // 由 /joint_states 订阅回调更新
    void update_state(double left_vel, double right_vel, double left_pos, double right_pos) {
        cached_state_.left_velocity = left_vel;
        cached_state_.right_velocity = right_vel;
        cached_state_.left_position = left_pos;
        cached_state_.right_position = right_pos;
    }

private:
    ChassisConfig cfg_;
    ChassisCommand last_cmd_;
    ChassisState cached_state_;
    double wheel_base_{0.3};
    double wheel_radius_{0.05};
};

}  // namespace chassis_protocol
```

---

## 二、Phase 3 VLA 训练数据飞轮

### 2.1 数据飞轮闭环架构（ASCII）

```
┌───────────────────────────────────────────────────────────────────────┐
│                     VLA 数据飞轮（仿真为核心）                           │
│                                                                       │
│  ┌──────────────────┐                                                  │
│  │  Isaac Sim       │  1. 遥操作/脚本演示                               │
│  │  (并行 1024 envs)│◄────────── SpaceMouse / VR Controller            │
│  │                  │                                                  │
│  │  + Domain        │  2. 自动 rollout（已有策略）                       │
│  │    Randomization │◄────────── 现有 VLA 模型推理                       │
│  └────────┬─────────┘                                                  │
│           │                                                            │
│           │  rosbag2 录制 (MCAP + zstd)                               │
│           ▼                                                            │
│  ┌──────────────────┐                                                  │
│  │  Phase 6         │  3. bag → LeRobot HDF5 转换                      │
│  │  rosbag2 管线     │     rosbag2_to_lerobot.py                        │
│  └────────┬─────────┘     数据质量过滤（EpisodeQualityChecker）          │
│           │                                                            │
│           │  LeRobot HDF5 数据集                                        │
│           ▼                                                            │
│  ┌──────────────────┐                                                  │
│  │  Phase 3         │  4. VLA 训练（π0 / OpenVLA / RDT）                │
│  │  VLA 训练管线     │     Sim(80%) + Real(20%) 混合                     │
│  │  (GPU 集群)       │     → 新模型 checkpoint                          │
│  └────────┬─────────┘                                                  │
│           │                                                            │
│           │  模型部署（TensorRT 量化）                                   │
│           ▼                                                            │
│  ┌──────────────────┐                                                  │
│  │  实机 / 仿真      │  5. 部署评估：成功率 / 鲁棒性                      │
│  │  评估环境         │     失败 episode → 触发新一轮仿真数据采集            │
│  │                  │                                                  │
│  │  chassis_protocol│  6. 实机数据 → nvblox → 数字孪生更新               │
│  │  + ros2_control  │◄────────── 实机 rosbag2 → Isaac Sim USD            │
│  └──────────────────┘                                                  │
│                                                                       │
│  ← ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ 闭环迭代 ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ → │
└───────────────────────────────────────────────────────────────────────┘
```

### 2.2 关键集成接口

| 阶段 | 输入 | 输出 | 工具 |
|------|------|------|------|
| 演示采集 | 遥操作指令 | rosbag2 (MCAP) | Isaac Sim + ros2 bag |
| 格式转换 | rosbag2 | LeRobot HDF5 | `rosbag2_to_lerobot.py` |
| 数据增强 | HDF5 raw | HDF5 augmented | torchvision + 关节噪声注入 |
| VLA 训练 | LeRobot HDF5 | `.pt` checkpoint | LeRobot train CLI |
| 推理部署 | 相机 + 关节状态 | 关节轨迹 | TensorRT + isaac_ros_tensor_rt |
| 评估反馈 | rollout bag | 成功率/失败分析 | PolicyRolloutRecorder |

---

## 三、Phase 6 数据录制对接

### 3.1 仿真与实机 topic 对齐

| Topic | 仿真（Isaac/Gazebo） | 实机（chassis_protocol） | 对齐方案 |
|-------|-------------------|------------------------|---------|
| `/joint_states` | 由 ros2_control JTC 发布 | 由 ChassisNode 发布 | ✅ 完全一致 |
| `/tf` | gz_ros2_control 广播 | robot_state_publisher | ✅ 相同 frame_id |
| `/camera/color/image_raw` | ros_gz_bridge 桥接 | RealSense 驱动发布 | ✅ 相同消息类型 |
| `/imu/data` | gz_ros2_control IMU | IMU 驱动发布 | ✅ 相同消息类型 |
| `/wrench` | FT broadcaster | FT 传感器驱动 | ✅ `WrenchStamped` |
| `/clock` | `/clock` topic | 系统时钟 | ⚠️ 需切换 `use_sim_time` |

### 3.2 格式一致性保证

```yaml
# 仿真/实机 rosbag2 录制的统一 RecordOptions
# 文件：config/recording_config.yaml

topics:
  - /joint_states
  - /joint_commands
  - /tf
  - /tf_static
  - /camera/color/image_raw
  - /camera/color/camera_info
  - /camera/depth/image_raw
  - /imu/data
  - /wrench
  - /odom
  - /cmd_vel

storage:
  storage_id: mcap
  storage_config:
    chunk_size: 4194304
    compression: zstd

# 实机专用（不含 /clock）
exclude_topics_real: [/clock]
# 仿真专用（含 /clock）
include_topics_sim: [/clock]
```

---

## 四、Phase 7 RT 对接

### 4.1 仿真进程 CPU 隔离（非 RT 核）

仿真进程（Isaac Sim / Gazebo）的 GPU 渲染线程及 ROS 2 Bridge 进程不得占用 RT 核心，否则会干扰 `controller_manager` 的 1kHz 实时控制环：

```bash
# /etc/systemd/system/gazebo_sim.service
[Unit]
Description=Gazebo Harmonic Simulation (Non-RT)
After=network.target

[Service]
Type=simple
# 限制 Gazebo 进程使用 CPU 0-3（RT 核心为 CPU 4-7）
ExecStartPre=/usr/bin/cgroup-setup.sh sim 0-3
ExecStart=/bin/bash -c \
  "taskset -c 0-3 gz sim -r /opt/robot/worlds/test_world.sdf"
CPUAffinity=0 1 2 3    # systemd 级别 CPU 亲和性
CPUSchedulingPolicy=other
CPUSchedulingPriority=0
Environment="ROS_DOMAIN_ID=1"
Environment="GZ_SIM_RESOURCE_PATH=/opt/robot/models"

[Install]
WantedBy=multi-user.target
```

### 4.2 use_sim_time 与 RT 时钟模型切换

```python
# launch/robot_bringup.launch.py
# 通过 sim_mode 参数自动切换时钟源

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sim_mode = LaunchConfiguration('sim_mode', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('sim_mode', default_value='false',
                              description='Use simulation time'),

        # 仿真模式：所有节点使用 /clock topic
        SetParameter(name='use_sim_time', value=True,
                     condition=IfCondition(sim_mode)),

        # 实机模式：使用系统 CLOCK_REALTIME（PREEMPT_RT 对齐）
        SetParameter(name='use_sim_time', value=False,
                     condition=UnlessCondition(sim_mode)),
    ])
```

**RT 时钟模型说明：**

| 模式 | 时钟源 | 时钟分辨率 | 适用场景 |
|------|-------|----------|---------|
| `use_sim_time=true` | `/clock` topic（仿真步进） | 仿真步长（典型 1ms） | 仿真/录制回放 |
| `use_sim_time=false` | `CLOCK_REALTIME` / `CLOCK_MONOTONIC` | 1ns（hrtimer） | 实机 PREEMPT_RT |
| 混合（HiL） | GPS PPS + PTP（IEEE 1588） | ~100ns | 硬件在环 |

---

## 五、整体仿真-实机联合部署图

```
数字孪生（Isaac Sim）↔ 实机（chassis_protocol + ros2_control）并行架构

┌──────────────────────────────────────────────────────────────────────────┐
│  主控 SoC（Jetson AGX Orin 64GB）                                          │
│                                                                          │
│  Core 0-1  ─────── 系统后台                                               │
│  Core 2-3  ─────── Gazebo Sim / Isaac Bridge（非 RT）                      │
│  Core 4    ─────── controller_manager RT 线程（SCHED_FIFO 90）            │
│  Core 5    ─────── ros2_control HW 线程（SCHED_FIFO 85）                  │
│  Core 6    ─────── RS485 驱动线程（SCHED_FIFO 80）                         │
│  Core 7    ─────── 感知推理节点（SCHED_OTHER）                              │
│                                                                          │
│  ┌───────────────────────────┐    ┌──────────────────────────────────┐   │
│  │  仿真路径（开发/训练）       │    │  实机路径（部署/生产）              │   │
│  │                           │    │                                  │   │
│  │  Isaac Sim (x86 workstation│    │  chassis_protocol                │   │
│  │  or container)            │    │  ChassisSystem                   │   │
│  │     │                     │    │     │                            │   │
│  │     │ OmniGraph ROS2 Bridge│    │     │ RS485 Transport           │   │
│  │     │ ros_gz_bridge        │    │     │ VersionNegotiator         │   │
│  │     ▼                     │    │     ▼                            │   │
│  │  ros2_control             │    │  ros2_control                   │   │
│  │  GazeboSystem Plugin      │    │  ChassisSystem Plugin           │   │
│  │  (mock 关节驱动)            │    │  (真实 RS485 通信)               │   │
│  └───────────────────────────┘    └──────────────────────────────────┘   │
│            │                                    │                        │
│            └────────────┬───────────────────────┘                        │
│                         ▼                                                │
│             统一 ROS 2 话题接口                                            │
│      /joint_states / /cmd_vel / /tf / /odom                             │
│             │                                                            │
│    ┌────────▼────────┐  ┌────────────────┐  ┌──────────────────────┐   │
│    │  Nav2 导航栈     │  │  MoveIt 2      │  │  VLA Policy          │   │
│    │  BT Navigator   │  │  move_group    │  │  (TensorRT 推理)      │   │
│    └─────────────────┘  └────────────────┘  └──────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 六、选型决策矩阵

| 使用场景 | Isaac Sim | Gazebo Harmonic | MuJoCo 3.x | PyBullet |
|---------|-----------|-----------------|------------|---------|
| **RL 大规模训练**（1024+ 并行） | ✅⭐⭐⭐⭐⭐（Isaac Lab GPU） | ❌ 进程级并行，慢 | ✅⭐⭐⭐（MJX GPU，规模较小） | ⚠️⭐⭐（慢，社区维护减少） |
| **ROS 2 功能测试 / CI** | ⚠️⭐⭐（需 RTX GPU，CI 成本高） | ✅⭐⭐⭐⭐⭐（CPU 即可，开源） | ⚠️⭐⭐（ROS bridge 不成熟） | ❌⭐（官方 ROS 2 支持弱） |
| **传感器仿真（相机/LiDAR）** | ✅⭐⭐⭐⭐⭐（RTX 光追，最真实） | ✅⭐⭐⭐⭐（GPU Lidar，相机栅格） | ⚠️⭐⭐（基础相机，无 LiDAR） | ⚠️⭐⭐（基础传感器） |
| **硬件在环（HiL）** | ⚠️⭐⭐（延迟较高） | ✅⭐⭐⭐⭐（`gz_ros2_control` 稳定） | ⚠️⭐⭐（需自定义） | ❌⭐ |
| **高精度接触力仿真** | ✅⭐⭐⭐⭐（PhysX 5） | ✅⭐⭐⭐（DART） | ✅⭐⭐⭐⭐⭐（TDM，学术最优） | ⚠️⭐⭐（简单接触） |
| **ARM64 / Jetson 部署** | ⚠️⭐⭐（x86+RTX 为主） | ✅⭐⭐⭐⭐⭐（官方支持） | ✅⭐⭐⭐⭐（跨平台） | ✅⭐⭐⭐ |
| **授权成本** | ⚠️⭐⭐（商业收费；研究免费） | ✅⭐⭐⭐⭐⭐（Apache 2.0） | ✅⭐⭐⭐⭐⭐（Apache 2.0） | ✅⭐⭐⭐⭐⭐（MIT） |
| **chassis_protocol 对接** | ✅ GazeboSystem / mock | ✅ gz_ros2_control 官方 | ⚠️ 需自定义 bridge | ❌ 不推荐 |

**本仓库推荐方案：**

- **功能测试 / CI**：Gazebo Harmonic + `gz_ros2_control` + `mock_components`
- **RL 训练 + VLA 数据生成**：Isaac Sim（工作站）+ Isaac Lab
- **高精度操控研究**：MuJoCo（接触力研究）+ Gazebo（ROS 2 集成）

---

## 七、数字孪生更新流程

### 7.1 实机数据 → Isaac Sim USD 自动更新

```
数字孪生更新流水线：

实机部署
  │
  │  ros2 bag record（实机工作数据）
  ▼
rosbag2 (MCAP)
  │
  │  nvblox_ros：从深度图 + 里程计重建 3D 场景
  ▼
nvblox Mesh (nvblox_msgs/Mesh)
  │
  │  mesh_to_usd.py：Mesh → OpenUSD Stage
  ▼
Isaac Sim USD Scene
  │
  │  系统辨识：从 bag 中提取 F/T + 关节轨迹
  │  → 更新 PhysX 关节摩擦/阻尼参数
  ▼
更新后的 Isaac Sim 数字孪生
  │
  │  基于更新后场景重新生成训练数据
  ▼
新一轮 VLA 训练（数据飞轮迭代）
```

### 7.2 参数迭代自动化脚本

```python
# scripts/update_digital_twin.py

import subprocess
import json
from pathlib import Path

def update_digital_twin(
    real_bag_path: str,
    usd_scene_path: str,
    output_usd_path: str,
):
    """从实机 rosbag2 更新 Isaac Sim 数字孪生场景"""

    # Step 1: nvblox 场景重建
    print("[1/4] Running nvblox reconstruction...")
    subprocess.run([
        "ros2", "run", "nvblox_ros", "nvblox_node",
        "--ros-args",
        "-p", "rosbag_path:=" + real_bag_path,
        "-p", "output_mesh_path:=/tmp/scene_mesh.ply",
        "-p", "voxel_size:=0.02",
    ], check=True)

    # Step 2: Mesh → USD 转换
    print("[2/4] Converting mesh to USD...")
    import omni.kit.commands as commands
    commands.execute('CreateMeshPrimWithDefaultXform',
                     prim_path='/World/SceneMesh',
                     prim_definition=str(Path("/tmp/scene_mesh.ply")))

    # Step 3: 系统辨识（关节参数）
    print("[3/4] Running system identification...")
    sys_id_result = run_system_identification(real_bag_path)
    update_physx_joint_params(usd_scene_path, sys_id_result)

    # Step 4: 保存更新后的 USD
    print("[4/4] Saving updated USD stage...")
    import omni.usd
    omni.usd.get_context().save_as_stage(output_usd_path)
    print(f"Digital twin updated: {output_usd_path}")
```
