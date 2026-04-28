# 参数系统（四）：高级模式与最佳实践

## 一、参数子命名空间与批量操作

```cpp
// ★ 使用点号分隔构建层次结构
declare_parameter("robot.arm.joint1.max_velocity", 2.0);
declare_parameter("robot.arm.joint2.max_velocity", 1.5);
declare_parameter("robot.base.max_linear_vel", 0.5);

// 批量读取子命名空间下的所有参数
std::map<std::string, double> arm_params;
get_parameters("robot.arm", arm_params);
// arm_params = {{"joint1.max_velocity": 2.0}, {"joint2.max_velocity": 1.5}}

// list_parameters 带 prefix 和 depth：
auto result = list_parameters({"robot.arm"}, 2);
// result.names = ["robot.arm.joint1.max_velocity", "robot.arm.joint2.max_velocity"]
// result.prefixes = ["robot.arm.joint1", "robot.arm.joint2"]
```

---

## 二、从 YAML 文件加载参数

```yaml
# config/pid_params.yaml
pid_controller:
  ros__parameters:           # ★ 固定键名 ros__parameters
    kp: 1.5
    ki: 0.05
    kd: 0.001
    limits:
      max_output: 100.0
      min_output: -100.0

# ★ 多节点 YAML（wildcard 节点名）
/**:                         # ★ 匹配所有节点
  ros__parameters:
    log_level: INFO
```

```python
# Launch 文件加载参数：
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

Node(
    package='my_package',
    executable='pid_controller',
    parameters=[
        os.path.join(get_package_share_directory('my_package'), 'config', 'pid_params.yaml'),
        {'extra_param': 42}  # 也可以内联字典
    ]
)
```

---

## 三、read_only 参数的用法

```cpp
// 只读参数：启动时从 YAML/命令行设置，运行时不可修改
rcl_interfaces::msg::ParameterDescriptor desc;
desc.read_only = true;
desc.description = "Robot model name, set at startup";

declare_parameter("robot_model", "ur5e", desc);

// 尝试修改时：
// ros2 param set /my_node robot_model "ur10e"
// → Error: Parameter 'robot_model' is read_only
// → on_set_parameters_callback 不会触发
```

---

## 四、参数类型转换细节

```cpp
// integer → double 的自动转换（兼容模式）
// 命令行：-p max_speed:=2  （整数）
// 声明：declare_parameter("max_speed", 2.5)  （double 类型）
// → RCL 自动将整数 2 转换为 double 2.0（兼容）

// ★ 数组参数
declare_parameter("joint_names", 
  std::vector<std::string>{"joint1", "joint2", "joint3"});
declare_parameter("joint_limits",
  std::vector<double>{1.0, 1.5, 2.0});

auto names = get_parameter("joint_names").as_string_array();
auto limits = get_parameter("joint_limits").as_double_array();

// ★ YAML 中的数组格式：
// joint_names: ["joint1", "joint2", "joint3"]
// joint_limits: [1.0, 1.5, 2.0]
```

---

## 五、参数系统性能考量

```
get_parameter()（本节点）：
  → 直接访问 std::map（加 mutex）
  → 极快：O(log N)，几微秒级

set_parameter()（本节点）：
  → 验证 + 回调 + map 更新 + 发布 /parameter_events
  → 稍慢：取决于回调复杂度，但通常 < 1ms

AsyncParametersClient（远程节点）：
  → 通过 Service 调用：网络 RTT + DDS 序列化
  → 同进程内：~几百微秒
  → 跨机器：几毫秒
  → 不适合高频调用

推荐模式：
  - 高频使用的参数：在本地缓存（declare → 成员变量），通过 callback 更新
  - 低频配置：可以每次 get_parameter() 或 AsyncParametersClient
  - 跨节点共享参数：考虑 parameter_blackboard 或 Topic 广播
```

---

## 六、参数系统与 ros2_control 集成

```cpp
// ros2_control 大量使用参数系统进行硬件配置
// 典型模式：

// hardware_interface/src/resource_manager.cpp
// 从参数文件加载硬件描述（URDF + hardware config）

// controller_manager 使用参数控制 controller 加载：
//   controller_manager.yaml：
//     controller_manager:
//       ros__parameters:
//         update_rate: 100  # Hz
//         joint_trajectory_controller:
//           type: joint_trajectory_controller/JointTrajectoryController
//
//         diff_drive_controller:
//           type: diff_drive_controller/DiffDriveController

// 动态加载 controller（通过 service + 参数联动）：
// ros2 service call /controller_manager/load_controller \
//   controller_manager_msgs/srv/LoadController \
//   "{name: 'my_controller'}"
```
