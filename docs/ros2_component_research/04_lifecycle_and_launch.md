# ROS2 Component（四）：LifecycleNode 与托管组件

## 一、LifecycleNode 概述

ROS2 提供了 `rclcpp_lifecycle::LifecycleNode`，它在普通 Node 基础上增加了**状态机管理**，是工业场景中最常用的 Component 模式。

### 状态机图

```
            configure()          activate()
Unconfigured ──────────► Inactive ──────────► Active
      ▲                      │                   │
      │      cleanup()        │ deactivate()      │
      └───────────────────────┘◄──────────────────┘
                               │
                         shutdown() ↓
                           Finalized
```

| 状态 | 描述 |
|------|------|
| Unconfigured | 初始状态，资源未分配 |
| Inactive | 已配置，但未激活（不处理数据） |
| Active | 正常运行状态 |
| Finalized | 已清理，不可恢复 |
| (ErrorProcessing) | 过渡期出错时的错误处理状态 |

---

## 二、LifecycleNode 源码结构

```
lifecycle/
├── rclcpp_lifecycle/
│   ├── include/rclcpp_lifecycle/
│   │   ├── lifecycle_node.hpp        # LifecycleNode 类声明
│   │   ├── node_interfaces/
│   │   │   └── lifecycle_node_interface.hpp
│   │   └── state.hpp                 # State 类（包含 id, label）
│   └── src/
│       ├── lifecycle_node.cpp        # 实现
│       └── state_machine_interface.cpp
```

---

## 三、LifecycleNode 关键源码

### 3.1 类声明（简化）

```cpp
// rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp

class LifecycleNode : public node_interfaces::LifecycleNodeInterface,
                      public rclcpp::node_interfaces::NodeBaseInterface,
                      // ... 其他接口
{
public:
  explicit LifecycleNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 用户需要覆写的回调（返回 SUCCESS / FAILURE / ERROR）
  virtual CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state);

  virtual CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  virtual CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  virtual CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state);

  virtual CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state);

  virtual CallbackReturn
  on_error(const rclcpp_lifecycle::State & previous_state);

  // 手动触发状态转换（用于测试或自主状态管理）
  const State & trigger_transition(const Transition & transition);
  
  // 当前状态查询
  const State & get_current_state() const;
};
```

### 3.2 状态转换触发机制

```cpp
// LifecycleNode 内部通过 ROS2 服务暴露生命周期管理接口
// 服务名格式：/<node_name>/change_state

// lifecycle_msgs/srv/ChangeState.srv
lifecycle_msgs/Transition transition
---
bool success

// lifecycle_msgs/srv/GetState.srv
---
lifecycle_msgs/State current_state
```

---

## 四、Lifecycle Component 示例

```cpp
// my_lifecycle_component.cpp

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("lifecycle_talker", options)
  {}

  // 配置阶段：分配资源
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    // 创建发布者（Inactive 状态下不激活）
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    RCLCPP_INFO(get_logger(), "Configured");
    return CallbackReturn::SUCCESS;
  }

  // 激活阶段：启动定时器
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // 激活 lifecycle publisher（开始接受消息）
    pub_->on_activate();
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        if (pub_->is_activated()) {
          auto msg = std_msgs::msg::String();
          msg.data = "Hello from lifecycle talker";
          pub_->publish(msg);
        }
      });
    return CallbackReturn::SUCCESS;
  }

  // 停用阶段：停止定时器
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  // 清理阶段：释放资源
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    pub_.reset();
    timer_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(LifecycleTalker)
```

---

## 五、LifecyclePublisher 特殊性

```cpp
// rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_publisher.hpp

// LifecyclePublisher 继承自普通 Publisher，但增加了激活状态检查
template<typename MessageT, typename Alloc>
class LifecyclePublisher : public rclcpp::Publisher<MessageT, Alloc>
{
public:
  void publish(std::unique_ptr<MessageT, Deleter> msg)
  {
    if (!is_activated_) {
      // ★ 处于 Inactive 状态时，丢弃消息（不发布）
      log_publisher_not_enabled();
      return;
    }
    rclcpp::Publisher<MessageT, Alloc>::publish(std::move(msg));
  }

  void on_activate() { is_activated_ = true; }
  void on_deactivate() { is_activated_ = false; }
  bool is_activated() const { return is_activated_; }

private:
  bool is_activated_{false};
};
```

---

## 六、通过 Launch 文件管理 Lifecycle Component

```python
# launch 文件示例（ROS2 Python Launch）
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, ComposableNode
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
import launch_ros.events.lifecycle

def generate_launch_description():
    # 创建容器，内含 lifecycle 组件
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_lifecycle_pkg',
                plugin='LifecycleTalker',
                name='lifecycle_talker',
            ),
        ],
    )
    
    # 在容器启动后，自动触发 configure
    configure_event = EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch_ros.event_handlers.matches_node_name(
                node_name='lifecycle_talker'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    return LaunchDescription([container, configure_event])
```

---

## 七、ComposableNode Launch API

```python
# 通过 LaunchFile 静态声明组合节点（推荐生产环境使用）
from launch_ros.actions import ComposableNodeContainer, ComposableNode

container = ComposableNodeContainer(
    name='image_processing_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',    # 多线程容器
    composable_node_descriptions=[
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            remappings=[('/image_raw', '/camera/image_raw')],
            parameters=[{'output_encoding': 'mono8'}],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            remappings=[('/image', '/debayer/image')],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ],
)
```

---

## 八、LoadableComponent vs ManuallyComposed

| 方式 | 优点 | 缺点 | 适用场景 |
|------|------|------|---------|
| ComponentManager 动态加载 | 运行时灵活，支持 `ros2 component` CLI | 依赖 DDS 服务通信（加载时） | 开发、调试、可重配置系统 |
| LaunchFile ComposableNode | 声明式、自动化、可重复 | 需重启才能更改组合 | 生产环境 |
| 手动静态组合（manual_composition） | 零开销，最简单 | 不支持运行时更改 | 性能极限优化场景 |

---

## 九、与 component 相关的 rcl 层接口

```c
// rcl/include/rcl/node_options.h

typedef struct rcl_node_options_s
{
  rcl_allocator_t allocator;
  bool use_global_arguments;
  rcl_arguments_t arguments;
  bool enable_rosout;
  bool use_intra_process_comms;   // ← 进程内通信标志
  bool enable_logger_service;
} rcl_node_options_t;
```

NodeOptions 在 `rclcpp::NodeOptions` 中封装此结构，最终传递给 `rcl_node_init()`。

---

## 十、关键调试命令速查

```bash
# 列出所有容器及其组件
ros2 component list

# 查看组件类型信息
ros2 component types

# 负载测试：验证进程内通信
ros2 run composition manual_composition

# 内省容器状态
ros2 node list | grep ComponentManager

# 运行时动态加载
ros2 component load /ComponentManager composition composition::Talker

# 查看组件参数
ros2 param list /talker

# 监控 lifecycle 节点状态
ros2 lifecycle get /lifecycle_talker
ros2 lifecycle set /lifecycle_talker configure
ros2 lifecycle set /lifecycle_talker activate
```
