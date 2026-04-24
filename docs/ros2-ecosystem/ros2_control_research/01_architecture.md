# ros2_control — 模块架构与核心数据结构

> 参考版本：ros2_control `humble`  
> 参考源码：`hardware_interface/`、`controller_manager/`、`controller_interface/`

---

## 一、包依赖关系

```
ros2_control（元包）
├── hardware_interface          ← HAL 抽象层（最核心）
│     ├── StateInterface
│     ├── CommandInterface
│     ├── ResourceManager
│     └── SystemInterface / ActuatorInterface / SensorInterface（基类）
├── controller_interface        ← 控制器基类
│     ├── ControllerInterface（继承 LifecycleNode）
│     └── ChainableControllerInterface
├── controller_manager          ← 运行时调度中枢
│     ├── ControllerManager（继承 rclcpp::Node）
│     └── spawner / unspawner CLI
├── transmission_interface      ← 传动比抽象（可选）
└── joint_limits                ← 关节限位（可选，通常由控制器使用）
```

独立仓库（不在 ros2_control 主仓）：

```
ros2_controllers          ← 官方内置控制器集合
gz_ros2_control           ← Gazebo 硬件插件
isaac_ros2_control        ← Isaac Sim 硬件插件
mock_components           ← 纯内存 mock（测试/仿真用）
```

---

## 二、核心数据结构

### 2.1 `Handle`（`StateInterface` / `CommandInterface` 的公共基类）

```cpp
// hardware_interface/include/hardware_interface/handle.hpp
class Handle {
  std::string prefix_name_;      // 通常是 joint 名称，如 "joint1"
  std::string interface_name_;   // 类型，如 "position" / "velocity" / "effort"
  double * value_ptr_;           // 指向 ResourceManager 中托管的 double
};
```

关键点：
- `StateInterface` 和 `CommandInterface` 都继承 `Handle`，区别仅在**所有权语义**
- `value_ptr_` 指向同一块内存中的 `double`：`hardware_interface` 的 `read()` 写入，控制器的 `update()` 读取（StateInterface）或写入（CommandInterface）
- 这块内存 **不在 DDS 通道上**，是进程内共享，零拷贝

### 2.2 `LoanedStateInterface` / `LoanedCommandInterface`

```cpp
// hardware_interface/include/hardware_interface/loaned_state_interface.hpp
class LoanedStateInterface {
  std::reference_wrapper<StateInterface> interface_;
  // 内部持有锁（可选，取决于实现）
  double get_value() const;
};

// hardware_interface/include/hardware_interface/loaned_command_interface.hpp
class LoanedCommandInterface {
  std::reference_wrapper<CommandInterface> interface_;
  void set_value(double value);
  double get_value() const;
};
```

关键点：
- `ResourceManager::claim_command_interfaces()` 返回 `LoanedCommandInterface` 列表
- **所有权互斥**：同一 `CommandInterface` 在同一时刻只能被一个控制器 claim
- 控制器 deactivate 时自动归还（RAII-like，通过 `ResourceManager` 追踪）

### 2.3 `HardwareInfo`（URDF 解析结果）

```cpp
// hardware_interface/include/hardware_interface/hardware_info.hpp
struct HardwareInfo {
  std::string name;                              // <ros2_control name="...">
  std::string type;                              // system / actuator / sensor
  std::string hardware_plugin_name;             // 插件类名
  std::map<std::string, std::string> hardware_parameters;  // <param>
  std::vector<ComponentInfo> joints;            // <joint> 列表
  std::vector<ComponentInfo> sensors;           // <sensor> 列表
  std::vector<ComponentInfo> gpios;             // <gpio> 列表（自定义 IO）
  std::string original_xml;                     // 原始 URDF 片段
};

struct ComponentInfo {
  std::string name;
  std::map<std::string, std::string> parameters;
  std::vector<InterfaceInfo> state_interfaces;    // <state_interface>
  std::vector<InterfaceInfo> command_interfaces;  // <command_interface>
};

struct InterfaceInfo {
  std::string name;              // "position" / "velocity" / "effort" ...
  std::string data_type;         // 默认 "double"
  std::string min;               // 可选
  std::string max;               // 可选
  std::string initial_value;     // 可选，启动时赋值
};
```

### 2.4 `ControllerInfo`（控制器元数据）

```cpp
// controller_manager/include/controller_manager/controller_manager.hpp（内部结构）
struct ControllerSpec {
  std::shared_ptr<controller_interface::ControllerInterfaceBase> c;
  std::shared_ptr<controller_manager::ControllerManagerNode> node;
  std::string type;              // 插件类名
  std::string name;              // 实例名
  lifecycle_msgs::msg::State state;
};
```

---

## 三、URDF `<ros2_control>` 标签结构

完整 URDF 片段示例（机械臂）：

```xml
<ros2_control name="MyArm" type="system">
  <hardware>
    <plugin>my_arm_hardware/MyArmSystem</plugin>
    <param name="can_interface">can0</param>
    <param name="update_rate">500</param>
  </hardware>

  <joint name="joint1">
    <command_interface name="position">
      <param name="min">-3.14159</param>
      <param name="max">3.14159</param>
    </command_interface>
    <command_interface name="velocity"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint2">
    <!-- 同上 -->
  </joint>

  <sensor name="ft_sensor">
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
  </sensor>

  <gpio name="digital_out">
    <command_interface name="pin0"/>
    <state_interface name="pin0"/>
  </gpio>
</ros2_control>
```

URDF 解析入口：

```
// hardware_interface/src/component_parser.cpp
parse_control_resources_from_urdf(urdf_string)
  → 找到所有 <ros2_control> 标签
  → 对每个标签：parse_hardware_info()
      → 解析 <hardware>/<plugin> → hardware_plugin_name
      → 解析 <hardware>/<param> → hardware_parameters
      → 解析 <joint> → ComponentInfo（含 state/command_interface）
      → 解析 <sensor> → ComponentInfo
      → 解析 <gpio> → ComponentInfo
  → 返回 vector<HardwareInfo>
```

---

## 四、接口命名约定

`StateInterface` / `CommandInterface` 的全名格式：

```
<prefix>/<interface_name>
例如：
  joint1/position
  joint1/velocity
  joint1/effort
  ft_sensor/force.x
  digital_out/pin0
```

标准接口名常量（`hardware_interface/types/hardware_interface_type_values.hpp`）：

```cpp
const char HW_IF_POSITION[] = "position";
const char HW_IF_VELOCITY[] = "velocity";
const char HW_IF_ACCELERATION[] = "acceleration";
const char HW_IF_EFFORT[] = "effort";
```

---

## 五、生命周期状态机（整体）

ros2_control 中存在**两套**生命周期状态机，但遵循同一个 `lifecycle_node` 模型：

### 5.1 硬件接口（HardwareInterface）生命周期

```
UNCONFIGURED ──on_configure()──► INACTIVE ──on_activate()──► ACTIVE
     ▲                               │                          │
     │                    on_cleanup()│              on_deactivate()
     │                               ▼                          │
     └────────────────────────── INACTIVE ◄────────────────────┘
```

对应回调：
| 回调 | 触发时机 | 典型操作 |
|------|---------|---------|
| `on_init(HardwareInfo)` | ResourceManager 加载时（非 lifecycle） | 解析参数、分配缓冲区 |
| `on_configure()` | configure 转换 | 开启通信、初始化驱动 |
| `on_activate()` | activate 转换 | 使能电机、开始周期性 read/write |
| `on_deactivate()` | deactivate 转换 | 关闭使能、停止收发 |
| `on_cleanup()` | cleanup 转换 | 关闭通信端口 |
| `on_shutdown()` | 紧急关机 | 同 on_deactivate + on_cleanup |

### 5.2 控制器（ControllerInterface）生命周期

与硬件接口完全对称，额外有：
- `on_configure()`：声明并 claim 所需的 state/command interfaces
- `on_activate()`：重置内部状态、使能输出
- `on_deactivate()`：停止输出（将 command interface 设为安全值）

---

## 六、ResourceManager 内部存储结构

```
ResourceManager
  ├── hardware_info_map_：map<string, HardwareInfo>        # 元数据
  ├── hardware_components_info_：map<string, ComponentData>  # 运行时组件
  │     ComponentData {
  │       shared_ptr<ActuatorInterface/SystemInterface/SensorInterface> instance
  │       vector<StateInterface> state_interfaces
  │       vector<CommandInterface> command_interfaces
  │       State lifecycle_state
  │     }
  ├── state_interface_map_：map<string, ref<StateInterface>>   # 全局名称索引
  ├── command_interface_map_：map<string, ref<CommandInterface>>
  └── claimed_command_interface_map_：set<string>          # 记录已被 claim 的接口
```

ResourceManager 是进程内**单例**（每个 CM 进程一个），是所有 read/write 操作的协调者。
