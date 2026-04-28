# 参数系统（一）：架构与类型系统

## 一、参数系统设计哲学

```
ROS2 参数 vs ROS1 参数服务器：
  ROS1：全局中心化参数服务器（rosmaster 管理），任何节点可读写任何参数
  ROS2：去中心化，每个节点自管参数（通过 Service 暴露），隔离性更强

核心原则：
  1. 节点自治：每个节点拥有自己的参数空间 /node_name/param_name
  2. 类型安全：参数有明确类型，不能运行时改变类型（除非 re-declare）
  3. 声明优先：必须先 declare 才能使用（Humble 前有 allow_undeclared 选项）
  4. 验证机制：set_parameter 时可运行用户验证回调
  5. 事件通知：参数变化时可触发回调（动态重配置）
```

---

## 二、参数类型系统

```cpp
// rcl_interfaces/msg/ParameterType.msg
uint8 PARAMETER_NOT_SET    = 0   // 未声明
uint8 PARAMETER_BOOL       = 1   // bool
uint8 PARAMETER_INTEGER    = 2   // int64_t
uint8 PARAMETER_DOUBLE     = 3   // double
uint8 PARAMETER_STRING     = 4   // std::string
uint8 PARAMETER_BYTE_ARRAY = 5   // std::vector<uint8_t>
uint8 PARAMETER_BOOL_ARRAY = 6   // std::vector<bool>
uint8 PARAMETER_INTEGER_ARRAY = 7 // std::vector<int64_t>
uint8 PARAMETER_DOUBLE_ARRAY  = 8 // std::vector<double>
uint8 PARAMETER_STRING_ARRAY  = 9 // std::vector<std::string>

// rclcpp::ParameterValue 封装：
class ParameterValue {
  rcl_interfaces::msg::ParameterValue value_;
  
  // 类型安全访问：
  bool get<bool>() const;
  int64_t get<int64_t>() const;
  double get<double>() const;
  std::string get<std::string>() const;
  std::vector<bool> get<std::vector<bool>>() const;
  // ...
  
  rclcpp::ParameterType get_type() const;
  std::string type_name() const;  // → "bool", "integer", "double", ...
};

// rclcpp::Parameter：名称 + 值
class Parameter {
  std::string name_;
  ParameterValue value_;
  
  // 构造：
  Parameter(const std::string & name, bool value);
  Parameter(const std::string & name, int value);
  Parameter(const std::string & name, double value);
  Parameter(const std::string & name, const std::string & value);
  // ...
};
```

---

## 三、ParameterDescriptor — 参数元数据

```cpp
// rcl_interfaces/msg/ParameterDescriptor.msg
string name
uint8  type
string description
string additional_constraints

# 数值范围（integer/double 类型适用）
bool read_only = false      # 是否只读（启动后不可修改）
bool dynamic_typing = false # 是否允许运行时改变类型（Humble+）

FloatingPointRange[] floating_point_range
  float64 from_value = -inf
  float64 to_value   = +inf
  float64 step = 0.0         # 0 = 无步长限制

IntegerRange[] integer_range
  int64 from_value
  int64 to_value
  uint64 step

// 使用示例：
rcl_interfaces::msg::ParameterDescriptor desc;
desc.description = "Frequency of the controller loop";
desc.floating_point_range.resize(1);
desc.floating_point_range[0].from_value = 1.0;
desc.floating_point_range[0].to_value = 1000.0;
desc.floating_point_range[0].step = 0.0;

declare_parameter("loop_frequency", 100.0, desc);
```

---

## 四、NodeParameters 内部存储

```cpp
// rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp

class NodeParameters : public NodeParametersInterface
{
  // ★ 参数存储：名称 → {ParameterValue, ParameterDescriptor}
  std::map<std::string, ParameterInfo> parameters_;
  
  struct ParameterInfo {
    ParameterValue value;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
  };
  
  // ★ 动态回调链（按注册顺序执行）
  using OnSetParametersCallbackType = 
    std::function<rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter> &)>;
  
  std::list<OnSetParametersCallbackType> on_set_parameters_callbacks_;
  
  // ★ 参数服务（6个 Service）
  std::shared_ptr<ParameterService> parameter_service_;
  
  mutable std::recursive_mutex mutex_;  // 并发保护（recursive 因为回调内可能再调用 set_parameter）
};
```

---

## 五、参数名称规则

```
绝对路径：/namespace/node_name/param_name
  例：/robot_arm/controller/max_velocity

相对路径（在 Node 内部 API 使用）：
  get_parameter("max_velocity")  → 自动加上节点完整名称前缀

参数子命名空间（分组）：
  declare_parameter("pid.kp", 1.0)   → 名称含点号
  declare_parameter("pid.ki", 0.1)
  declare_parameter("pid.kd", 0.01)
  
  → 可通过 list_parameters(["pid"], 1) 查询所有 pid.* 参数

数组元素：
  参数本身支持数组类型，不用 param[0] 格式
  declare_parameter("joint_names", std::vector<std::string>{"joint1","joint2"})
```

---

## 六、参数覆盖（Parameter Overrides）

```cpp
// 命令行传入参数（启动时覆盖默认值）：
// ros2 run my_package my_node --ros-args -p max_velocity:=2.5

// Launch 文件中：
// Node(parameters=[{"max_velocity": 2.5}])

// 参数文件（YAML）：
// ros2 run my_package my_node --ros-args --params-file config.yaml

// YAML 格式：
// my_node:
//   ros__parameters:
//     max_velocity: 2.5
//     pid:
//       kp: 1.0
//       ki: 0.1

// 内部处理（rcl 层）：
// rcl_arguments_t 解析命令行，存储参数覆盖（parameter overrides）
// declare_parameter 时：
//   1. 检查 parameter_overrides_ 是否有同名覆盖
//   2. 有则使用覆盖值（而非 default_value）
//   3. 无则使用 default_value

// 在 NodeParameters::declare_parameter() 中：
auto it = parameter_overrides_.find(name);
if (it != parameter_overrides_.end()) {
  initial_value = it->second;  // 使用命令行/文件指定的值
} else {
  initial_value = default_value;
}
```

---

## 七、全局参数文件与 parameter_blackboard

```
Node("global_parameters")：
  → 不是内置的，但有 rclcpp_components 实现

parameter_blackboard（rclcpp_components 包）：
  → 一个特殊节点，接受任意参数设置
  → 其他节点可通过 AsyncParametersClient 读取
  → 实现跨节点参数共享

AsyncParametersClient：
  → 远程访问其他节点的参数（通过 Service 调用）
  → 异步 API：get_parameters(), set_parameters(), list_parameters()
```

---

## 八、参数服务 6 个 Service 概述

```
Service 名称（相对于节点）：
  ~/get_parameters
    请求：string[] names
    响应：ParameterValue[] values

  ~/get_parameter_types
    请求：string[] names
    响应：uint8[] types

  ~/set_parameters
    请求：Parameter[] parameters
    响应：SetParametersResult[] results  # 每个参数独立结果

  ~/set_parameters_atomically
    请求：Parameter[] parameters
    响应：SetParametersResult result     # 全部成功或全部失败

  ~/describe_parameters
    请求：string[] names
    响应：ParameterDescriptor[] descriptors

  ~/list_parameters
    请求：string[] prefixes, uint64 depth
    响应：ListParametersResult { string[] names, string[] prefixes }
```
