# ROS2 参数系统深度调研 - 目录

## 文件列表

| 文件 | 内容 |
|------|------|
| [01_overview.md](./01_overview.md) | 参数系统架构、类型系统、三层结构 |
| [02_parameter_service.md](./02_parameter_service.md) | ParameterService 源码：6个 Service 实现 |
| [03_declare_and_get.md](./03_declare_and_get.md) | declare_parameter / get_parameter 完整源码 |
| [04_dynamic_callback.md](./04_dynamic_callback.md) | 动态参数回调（on_set_parameters_callback） |
| [05_summary.md](./05_summary.md) | 速查手册、参数类型、常见模式 |

## 核心架构

```
应用层：
  Node::declare_parameter<T>(name, default)
  Node::get_parameter<T>(name) → ParameterValue
  Node::set_parameter(Parameter) → SetParametersResult
  Node::add_on_set_parameters_callback(callback)

中间层（rclcpp）：
  NodeParameters（实现 NodeParametersInterface）
    ├── 存储：std::map<string, ParameterValue>
    ├── 服务：ParameterService（6个 rcl_service）
    └── 回调链：on_set_parameters_callbacks_

底层（rcl/rmw）：
  参数通过 Service 实现（无专用 rcl 基础设施）
  6个 Service Topics：
    /node_name/get_parameters
    /node_name/get_parameter_types
    /node_name/set_parameters
    /node_name/set_parameters_atomically
    /node_name/describe_parameters
    /node_name/list_parameters
```
