# ros2_control 端到端时序图

> 本文以**具身智能机械臂**为例，描述从启动到稳态实时循环、再到关机的完整时序。

---

## 一、启动时序（T=0 到首次实时循环）

```
time →

[launch 进程]
  1. xacro 处理 URDF (Xacro → XML)
  2. robot_state_publisher 启动，发布 /robot_description + /tf_static
  3. controller_manager (ros2_control_node) 启动

[ControllerManager 初始化]
  4. CM 接收 robot_description 参数
  5. ResourceManager::load_urdf(urdf)
      5a. component_parser::parse_control_resources_from_urdf()
            → 解析所有 <ros2_control> 标签 → vector<HardwareInfo>
      5b. pluginlib ClassLoader::createSharedInstance("my_robot_hardware/MyRobotSystem")
            → dlopen(libmy_robot_hardware.so)
            → 构造 MyRobotSystem 对象
      5c. hw->on_init(hardware_info)
            → 解析 CAN 接口、波特率等参数
            → 分配 position_states_[], velocity_states_[], command_interfaces_[] 缓冲区
      5d. hw->on_configure()
            → 打开 CAN 总线
            → 等待所有关节节点上线（NMT discovery）
      5e. hw->export_state_interfaces()
            → 返回 [{joint1/position, &pos[0]}, {joint1/velocity, &vel[0]}, ...]
      5f. hw->export_command_interfaces()
            → 返回 [{joint1/position, &cmd[0]}, ...]
      5g. RM 将接口存入 state_interface_map_ / command_interface_map_
  6. CM 创建管理 Service（load/switch/list...）
  7. CM 创建实时定时器（update_rate=500Hz → 2ms 周期）
      → Timer 此时已就绪，但还没有激活的控制器

[控制器加载（spawner 进程）]
  8. spawner 调用 /controller_manager/load_controller (joint_state_broadcaster)
      8a. CM: pluginlib 实例化 JointStateBroadcaster
      8b. CM: controller->init(node, urdf, update_rate)
      8c. CM 调用 configure_controller(joint_state_broadcaster)
            → controller->on_configure()
            → CM 调用 RM::claim_state_interfaces(["joint1/position", "joint1/velocity", ...])
               → LoanedStateInterface 列表存入 controller.state_interfaces_
  9. spawner 调用 /switch_controller（activate: joint_state_broadcaster）
      9a. CM: 切换到 ACTIVATING
      9b. controller->on_activate()
      9c. CM: 更新 rt_controllers_snapshot_ (原子替换)
      9d. 控制器进入 ACTIVE 状态

  10. 同样流程加载并激活 joint_trajectory_controller

[首次实时循环]
  11. 定时器触发 read_update_write(time, period)
```

---

## 二、稳态实时循环时序（2ms 周期，500Hz）

```
T=0ms                               T=2ms
│                                   │
▼                                   ▼
┌─────┬──────────────────────┬──────┐
│READ │       UPDATE          │WRITE │
│ ~50µs│      ~300µs          │ ~50µs│
└─────┴──────────────────────┴──────┘

READ 阶段（ResourceManager::read）：
  ├── MyRobotSystem::read()
  │     ├── CAN 读取（或上次中断的缓冲区）
  │     ├── position_states_[i] = can_feedback[i].position
  │     ├── velocity_states_[i] = can_feedback[i].velocity
  │     └── effort_states_[i]   = can_feedback[i].current * torque_const
  └── （其他 HW 组件，如 FT 传感器）
        └── FTSensorInterface::read()
              └── rs485_.read_wrench() → fx_, fy_, fz_, tx_, ty_, tz_

UPDATE 阶段（update_controllers）：
  控制器 1：JointStateBroadcaster::update()
    ├── 读取所有 LoanedStateInterface（position/velocity/effort）
    ├── 填充 sensor_msgs::JointState msg
    └── rt_pub_->trylock() → unlockAndPublish()
        → /joint_states topic（DDS，异步发布）

  控制器 2：JointTrajectoryController::update()
    ├── 从 RealtimeBuffer 取当前轨迹指令
    ├── 时间插值：cubic spline → goal_position[i], goal_velocity[i]
    ├── PID 计算：
    │     error[i] = goal_position[i] - state_interfaces_[position].get_value()
    │     d_error[i] = goal_velocity[i] - state_interfaces_[velocity].get_value()
    │     command[i] = kp*error + kd*d_error + ki*∫error（预积分 clamp）
    ├── command_interfaces_[i].set_value(command[i])   ← 写入共享内存
    └── 检查轨迹是否完成 → 发布 Action result（异步）

WRITE 阶段（ResourceManager::write）：
  └── MyRobotSystem::write()
        ├── 读取 command_interfaces_[i].get_value()  ← 从共享内存取
        ├── 打包 CAN 帧（位置控制帧格式）
        └── CAN 发送到各关节电机驱动器
```

---

## 三、MoveIt 2 规划请求时序

```
用户 / VLA 输出
  → MoveIt 2 Python API：move_group.plan(target_pose)

  ── 规划阶段（非实时，数十~数百毫秒）──────────────────────
  1. MoveGroupNode 收到请求
  2. planning_scene_monitor 更新 PlanningScene（基于最新 /joint_states）
  3. OMPL Planner::plan(start_state, goal_state)
     → 采样 + 碰撞检测（FCL）→ 生成关节轨迹（多个路径点）
  4. 轨迹时间参数化（TOTG / Ruckig）→ 加入时间戳 + 速度 + 加速度
  5. 返回 JointTrajectory

  ── 执行阶段（发给 JTC）──────────────────────────────────
  6. TrajectoryExecutionManager → ActionClient::send_goal()
     → /joint_trajectory_controller/follow_joint_trajectory (FollowJointTrajectory)
  7. JTC::goal_callback()（非实时）
     → 验证关节名称、时间戳
     → 存入 RealtimeBuffer<TrajectoryMsg>（原子替换）
  8. 下一个 CM 实时循环 tick：
     → JTC::update() 读取新轨迹，开始执行
  9. 每个 tick：插值 + PID + 写 command_interfaces
  10. 轨迹完成：JTC 发布 FollowJointTrajectory Result → MoveIt 2
```

---

## 四、控制器切换时序（非实时）

```
外部请求：ros2 service call /switch_controller
  （stop: [joint_trajectory_controller], start: [forward_command_controller]）

  [非实时线程，CM Service callback]
  1. 参数验证（控制器名称、严格性模式 STRICT/BEST_EFFORT）
  2. 检查接口冲突（forward_command_controller 需要 joint1/position，当前被 JTC 占用）
  3. 触发 JTC deactivate：
       JTC::on_deactivate()
         → command_interfaces_ 设为 NaN（安全值）
       RM::release_command_interfaces(["joint1/position", "joint2/position", ...])
         → claimed_command_interface_map_.erase(...)
  4. 触发 FCC configure：
       RM::claim_command_interfaces(["joint1/position", ...])
         → claimed_command_interface_map_.insert(...)
       FCC::on_configure() / on_activate()
  5. 构造新 rt_controllers_snapshot_（移除 JTC，加入 FCC）
  6. 原子替换：
       rt_controllers_snapshot_.store(new_snapshot, memory_order_release)

  [下一个实时循环 tick]
  7. CM 的实时线程加载新 snapshot：
       auto snapshot = rt_controllers_snapshot_.load(memory_order_acquire)
  8. FCC 开始在 update() 中执行（JTC 不再被调用）
```

---

## 五、关机时序

```
[SIGINT 或 ros2 lifecycle set /controller_manager shutdown]
  1. CM 接收关机信号
  2. 非实时线程：遍历所有 ACTIVE 控制器，依次 deactivate → cleanup
       controller->on_deactivate() → controller->on_cleanup()
  3. 非实时线程：遍历所有 ACTIVE 硬件，依次 deactivate → cleanup
       hw->on_deactivate()    ← 关闭电机使能
       hw->on_cleanup()       ← 关闭 CAN 总线、释放文件描述符
  4. ResourceManager 析构，释放所有 shared_ptr
  5. pluginlib ClassLoader 析构，卸载 .so
  6. CM 节点关闭
```

---

## 六、错误恢复时序

```
场景：MyRobotSystem::read() 连续 3 次返回 return_type::ERROR

  实时循环内：
  1. CM 检测到 read_failures（hardware_component = "MyRobot"）
  2. 超过阈值（3次）→ 标记该硬件为 ERROR 状态
     （实时线程中只设标志，不执行耗时操作）

  非实时线程（CM 内部 error_callback）：
  3. 调用 RM::set_component_state("MyRobot", ERROR_STATE)
       → hw->on_error()
           ← 尝试重新初始化 CAN 总线
           → 成功：返回 INACTIVE
           → 失败：保持 ERROR
  4. 依赖该硬件的控制器（JTC）被强制 deactivate
  5. 向上层发布 /diagnostics 错误消息（通过 hardware_state_broadcaster）
  6. 上层 BehaviorTree 检测到错误 → 触发恢复行为或急停
```

---

## 七、关键时延数值参考（具身智能机械臂）

| 路径段 | 典型延迟 |
|--------|---------|
| 传感器 → `read()` 完成 | 50~200 µs（CAN 125kbps）|
| `update()`（6 DOF JTC PID） | 50~100 µs |
| `write()` → CAN 发送完成 | 50~200 µs |
| **总控制环延迟（最坏）** | < 1 ms（500 Hz 时） |
| MoveIt 2 规划（OMPL） | 50~500 ms |
| `FollowJointTrajectory` Action 响应 | < 5 ms（到 JTC 更新 buffer） |
| VLA 推理（本机 GPU） | 10~200 ms |
