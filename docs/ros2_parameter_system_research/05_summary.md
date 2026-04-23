# 参数系统速查总结

## 核心 API 速查

| 操作 | 代码 |
|------|------|
| 声明参数 | `declare_parameter("name", default_value)` |
| 声明带描述符 | `declare_parameter("name", value, descriptor)` |
| 读取参数 | `get_parameter("name").as_double()` |
| 读取带默认值 | `get_parameter_or("name", fallback)` |
| 设置参数 | `set_parameter(rclcpp::Parameter("name", value))` |
| 批量设置（原子） | `set_parameters_atomically({...})` |
| 注册动态回调 | `add_on_set_parameters_callback(cb)` |
| 远程读取 | `AsyncParametersClient::get_parameters({"name"})` |

## 参数类型与 as_xxx() 方法

| 类型 | as_xxx() | YAML 格式 |
|------|---------|-----------|
| bool | `as_bool()` | `true/false` |
| int64 | `as_int()` | `42` |
| double | `as_double()` | `3.14` |
| string | `as_string()` | `"hello"` |
| bool[] | `as_bool_array()` | `[true, false]` |
| int64[] | `as_integer_array()` | `[1, 2, 3]` |
| double[] | `as_double_array()` | `[1.0, 2.5]` |
| string[] | `as_string_array()` | `["a", "b"]` |

## SetParametersResult 返回值约定

```cpp
// 验证通过：
result.successful = true;
result.reason = "";  // 或简短描述

// 验证失败：
result.successful = false;
result.reason = "kp must be non-negative";  // 详细原因
// → set_parameter() 调用方会收到此结果，参数不会被修改
```

## YAML 参数文件结构

```yaml
node_name:           # 节点名（不含命名空间）
  ros__parameters:   # 固定关键字
    param1: value1
    nested:
      param2: value2
```

## 调试命令

```bash
ros2 param list /node          # 列出所有参数
ros2 param get /node param     # 读取
ros2 param set /node param val # 设置
ros2 param dump /node          # 导出 YAML
ros2 param load /node file.yaml # 加载 YAML
ros2 param describe /node param # 查看描述符
```

## 常见坑

| 问题 | 原因 | 解决 |
|------|------|------|
| `ParameterNotDeclaredException` | 使用前未 declare | 先 `declare_parameter` |
| 动态回调不触发 | handle 没有保存 | 用成员变量持有 handle |
| 命令行值不生效 | 参数名写错 | 检查大小写/点号/格式 |
| 类型不匹配 | YAML integer vs double | 在 YAML 中加 `.0` 如 `42.0` |
| 远程设置无响应 | Service 未启动 | 检查 `wait_for_service` |
