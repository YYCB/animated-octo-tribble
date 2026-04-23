# ROS2 Component 知识总结与速查手册

## 一、核心概念一览

| 概念 | 描述 | 关键类/宏 |
|------|------|----------|
| Component | 可动态加载的节点单元 | `rclcpp::Node` + `RCLCPP_COMPONENTS_REGISTER_NODE` |
| ComponentManager | 管理组件加载/卸载的节点 | `rclcpp_components::ComponentManager` |
| NodeFactory | 组件实例化工厂 | `rclcpp_components::NodeFactoryTemplate<T>` |
| component_container | 组件宿主进程 | `component_container` / `component_container_mt` |
| IntraProcessManager | 进程内零拷贝通信 | `rclcpp::IntraProcessManager` |
| Executor | 事件循环调度器 | `SingleThreadedExecutor` / `MultiThreadedExecutor` |
| CallbackGroup | 回调并发控制 | `MutuallyExclusive` / `Reentrant` |
| LifecycleNode | 带状态机的 Component | `rclcpp_lifecycle::LifecycleNode` |

---

## 二、开发 Component 的完整步骤

### Step 1: 实现组件类（.hpp/.cpp）
```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class MyComponent : public rclcpp::Node {
public:
  explicit MyComponent(const rclcpp::NodeOptions & options)
  : Node("my_component", options) { /* ... */ }
};
RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

### Step 2: CMakeLists.txt
```cmake
add_library(my_component SHARED my_component.cpp)
ament_target_dependencies(my_component rclcpp rclcpp_components)
rclcpp_components_register_nodes(my_component "MyComponent")
install(TARGETS my_component LIBRARY DESTINATION lib)
```

### Step 3: 运行
```bash
# 终端1：启动容器
ros2 run rclcpp_components component_container

# 终端2：加载组件
ros2 component load /ComponentManager my_pkg MyComponent
```

---

## 三、常见陷阱

| 陷阱 | 原因 | 解决方案 |
|------|------|---------|
| 进程内通信不生效 | NodeOptions 未设置 use_intra_process_comms | 显式传入 extra_arguments 或 NodeOptions |
| Component 加载失败 | .so 未安装到 ament 索引路径 | `colcon build --symlink-install` 后 source install/setup.bash |
| 多线程 executor 数据竞争 | 共享状态无保护 | 使用 MutuallyExclusive CallbackGroup |
| LifecyclePublisher 不发布 | 未调用 on_activate() | 在 on_activate() 回调中调用 pub_->on_activate() |
| 构造函数签名错误 | NodeFactoryTemplate 模板要求 NodeOptions 参数 | 构造函数第一个参数必须是 `const rclcpp::NodeOptions &` |

---

## 四、源码文件速查索引

```
关键文件速查（GitHub: ros2/rclcpp）
─────────────────────────────────────────────────────────────

Component 框架核心：
  rclcpp_components/src/component_manager.cpp      ★★★★★
  rclcpp_components/include/rclcpp_components/component_manager.hpp
  rclcpp_components/include/rclcpp_components/node_factory.hpp
  rclcpp_components/include/rclcpp_components/register_node_macro.hpp

进程内通信：
  rclcpp/src/rclcpp/intra_process_manager.cpp      ★★★★★
  rclcpp/include/rclcpp/intra_process_manager.hpp
  rclcpp/src/rclcpp/publisher_base.cpp
  rclcpp/src/rclcpp/subscription_base.cpp

Executor：
  rclcpp/src/rclcpp/executors/single_threaded_executor.cpp   ★★★★
  rclcpp/src/rclcpp/executors/multi_threaded_executor.cpp    ★★★★
  rclcpp/src/rclcpp/executor.cpp                             ★★★★★

动态加载：
  class_loader/src/class_loader.cpp                ★★★★
  class_loader/src/class_loader_imp.cpp

Lifecycle：
  rclcpp_lifecycle/src/lifecycle_node.cpp          ★★★★
  rclcpp_lifecycle/src/state_machine_interface.cpp

官方示例：
  ros2/demos/composition/src/                      ★★★
```

---

## 五、Component 与微服务架构的类比

```
微服务（Microservice）         ROS2 Component
─────────────────────────    ─────────────────────────
服务进程（Service Process）   → Component 实例（Node）
API Gateway                  → ComponentManager（负责路由加载请求）
服务注册（Service Registry）  → ament_index（plugin 注册表）
服务容器（Docker）            → component_container 进程
进程间通信（HTTP/gRPC）       → DDS（进程间）
进程内调用（方法调用）         → IntraProcessManager（进程内）
健康检查 / 就绪探针           → LifecycleNode 状态机
热部署（Hot Reload）          → 运行时 load/unload component
```

---

## 六、参考链接

- **设计文档**：https://design.ros2.org/articles/intraprocess_communications.html
- **官方教程**：https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html
- **源码仓库**：https://github.com/ros2/rclcpp/tree/rolling/rclcpp_components
- **Demos**：https://github.com/ros2/demos/tree/rolling/composition
- **class_loader**：https://github.com/ros/class_loader
- **Lifecycle**：https://design.ros2.org/articles/node_lifecycle.html
