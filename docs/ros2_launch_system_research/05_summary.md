# Launch 系统速查总结

## 核心 Action 速查

| Action | 用途 | 包 |
|--------|------|-----|
| `Node` | 启动 ROS2 节点 | launch_ros |
| `ExecuteProcess` | 运行任意进程 | launch |
| `IncludeLaunchDescription` | 嵌套 Launch 文件 | launch |
| `DeclareLaunchArgument` | 声明启动参数 | launch |
| `GroupAction` | 带作用域的 Action 组 | launch_ros |
| `TimerAction` | 延迟执行 | launch |
| `LogInfo` | 打印日志 | launch |
| `RegisterEventHandler` | 注册事件响应 | launch |
| `Shutdown` | 关闭 Launch | launch |
| `SetEnvironmentVariable` | 设置环境变量 | launch |
| `OpaqueFunction` | 动态生成 Actions | launch |
| `ComposableNodeContainer` | 组件容器 | launch_ros |

## 核心 Substitution 速查

| Substitution | 功能 | 示例 |
|-------------|------|------|
| `LaunchConfiguration('x')` | 读取参数 | `LaunchConfiguration('use_sim_time')` |
| `FindPackageShare('pkg')` | 包 share 目录 | `FindPackageShare('nav2_bringup')` |
| `PathJoinSubstitution([...])` | 路径拼接 | `PathJoinSubstitution([FindPackageShare('p'), 'cfg', 'a.yaml'])` |
| `EnvironmentVariable('X')` | 环境变量 | `EnvironmentVariable('ROS_DOMAIN_ID')` |
| `PythonExpression('...')` | Python 表达式 | `PythonExpression(["'", LaunchConfiguration('m'), "' == 'sim'"])` |

## 核心 Condition 速查

| Condition | 说明 |
|-----------|------|
| `IfCondition(sub)` | sub 值为 "true"/"1"/"yes"/"on" 时执行 |
| `UnlessCondition(sub)` | sub 值不为 true 时执行 |
| `LaunchConfigurationEquals('x', 'val')` | 参数等于指定值 |
| `LaunchConfigurationNotEquals('x', 'val')` | 参数不等于指定值 |

## 核心 EventHandler 速查

| EventHandler | 触发时机 |
|-------------|---------|
| `OnProcessExit(target, on_exit)` | 目标进程退出 |
| `OnProcessStart(target, on_start)` | 目标进程启动 |
| `OnShutdown(on_shutdown)` | LaunchService 关闭 |

## 标准 .launch.py 模板

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 声明参数
    use_sim = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    # 2. 读取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 3. 定义节点
    my_node = Node(
        package='my_pkg',
        executable='my_exe',
        name='my_node',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[('input_topic', '/remapped_input')],
        output='screen',
    )
    
    # 4. 组装
    return LaunchDescription([
        use_sim,
        my_node,
        RegisterEventHandler(
            OnProcessExit(target_action=my_node, on_exit=[Shutdown()])
        ),
    ])
```

## 常用命令

```bash
# 运行 launch 文件
ros2 launch my_package my.launch.py

# 传入参数
ros2 launch my_package my.launch.py use_sim_time:=true robot_model:=ur10e

# 查看参数列表
ros2 launch my_package my.launch.py --show-args

# 调试模式（打印详细信息）
ros2 launch -d my_package my.launch.py

# 打印 launch 文件内容（不执行）
ros2 launch -p my_package my.launch.py
```

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| `LaunchConfiguration not set` | 未 `DeclareLaunchArgument` | 添加声明或提供 `default` |
| 节点没有启动 | condition 不满足 | 检查 IfCondition 的值 |
| 参数文件找不到 | `FindPackageShare` 找错包 | 检查包名和安装位置 |
| 节点退出后无法重启 | 需要 `RegisterEventHandler` | 用 `OnProcessExit` 重新添加节点 |
| `PythonExpression` 报错 | 表达式语法错误 | 检查引号和运算符 |
