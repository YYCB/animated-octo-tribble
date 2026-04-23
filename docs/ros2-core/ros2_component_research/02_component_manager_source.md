# ROS2 Component 源码分析（二）：ComponentManager 详解

## 一、ComponentManager 概述

`ComponentManager` 是 `rclcpp_components` 包的核心类，它本身就是一个 `rclcpp::Node`。它负责：
1. 监听来自外部（如 `ros2 component load` CLI）的服务请求
2. 动态加载 `.so` 共享库
3. 实例化 Component 节点
4. 将节点注册到 Executor

---

## 二、ComponentManager 源码解析

### 2.1 头文件：`rclcpp_components/include/rclcpp_components/component_manager.hpp`

```cpp
// 关键成员变量（精简版）
class ComponentManager : public rclcpp::Node
{
public:
  // 构造函数接受 executor 弱引用，用于将加载的节点添加到同一 executor
  explicit ComponentManager(
    std::weak_ptr<rclcpp::Executor> executor,
    std::string node_name = "_ComponentManager",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()
      .start_parameter_services(false)
      .start_parameter_event_publisher(false));

  virtual ~ComponentManager();

protected:
  // 处理 LoadNode 服务请求
  virtual void
  on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response);

  // 处理 UnloadNode 服务请求
  virtual void
  on_unload_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response);

  // 处理 ListNodes 服务请求
  virtual void
  on_list_nodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response);

  // 通过 class_loader 加载组件
  virtual std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const ComponentResource & resource);

private:
  std::weak_ptr<rclcpp::Executor> executor_;

  // 已加载组件的注册表
  std::unordered_map<uint64_t, NodeInstanceWrapper> node_wrappers_;

  // 三个 ROS 服务
  rclcpp::Service<LoadNode>::SharedPtr     load_node_srv_;
  rclcpp::Service<UnloadNode>::SharedPtr   unload_node_srv_;
  rclcpp::Service<ListNodes>::SharedPtr    list_nodes_srv_;

  // 类加载器实例列表（每个 .so 对应一个）
  std::vector<std::shared_ptr<class_loader::ClassLoader>> loaders_;
};
```

---

### 2.2 关键方法：`on_load_node()` 实现解析

```cpp
// 源码路径：rclcpp_components/src/component_manager.cpp
void ComponentManager::on_load_node(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<LoadNode::Request> request,
  std::shared_ptr<LoadNode::Response> response)
{
  (void)request_header;
  
  // Step 1: 根据 package_name 找到 .so 路径
  // 使用 ament_index_cpp::get_resource() 查询安装路径
  auto resources = get_component_resources(request->package_name);
  
  // Step 2: 在已注册资源中找到 plugin_name 对应的工厂
  for (auto & resource : resources) {
    if (resource.first == request->plugin_name) {
      // Step 3: 创建/复用 ClassLoader
      auto factory = create_component_factory(resource);
      if (factory) {
        // Step 4: 构建 NodeOptions（传入 remap 规则、参数等）
        auto options = rclcpp::NodeOptions();
        // ...（处理 request->remap_rules, extra_arguments）
        
        // Step 5: 通过工厂实例化节点
        auto wrapper = factory->create_node_instance(options);
        
        // Step 6: 分配唯一 ID，注册到 node_wrappers_
        uint64_t id = get_next_id();
        node_wrappers_[id] = wrapper;
        
        // Step 7: 将节点添加到共享 Executor
        if (auto exec = executor_.lock()) {
          exec->add_node(wrapper.get_node_base_interface(), true);
        }
        
        response->unique_id = id;
        response->full_node_name = wrapper.get_node_base_interface()->get_fully_qualified_name();
        response->success = true;
        return;
      }
    }
  }
  response->success = false;
  response->error_message = "Failed to find class with the requested plugin name.";
}
```

**关键调用链**：
```
ros2 component load <container> <pkg> <plugin>
  → ros2cli → ComponentManager::on_load_node()
    → ament_index_cpp::get_resource()         # 查 ament 索引
    → class_loader::ClassLoader(lib_path)      # dlopen .so
    → factory->create_node_instance(options)   # 调用注册的工厂函数
    → executor_->add_node()                    # 加入执行器
```

---

### 2.3 NodeFactory 接口

```cpp
// rclcpp_components/include/rclcpp_components/node_factory.hpp

class NodeFactory
{
public:
  virtual ~NodeFactory() {}

  // 核心接口：用给定 options 创建节点实例
  virtual NodeInstanceWrapper
  create_node_instance(const rclcpp::NodeOptions & options) = 0;
};
```

用户组件通过宏 `RCLCPP_COMPONENTS_REGISTER_NODE` 自动生成 `NodeFactory` 的具体子类：

```cpp
// rclcpp_components/include/rclcpp_components/register_node_macro.hpp
#define RCLCPP_COMPONENTS_REGISTER_NODE(NodeClass)                        \
  CLASS_LOADER_REGISTER_CLASS(                                             \
    rclcpp_components::NodeFactoryTemplate<NodeClass>,                     \
    rclcpp_components::NodeFactory)
```

`NodeFactoryTemplate` 的实现：

```cpp
template<class NodeT>
class NodeFactoryTemplate : public NodeFactory
{
public:
  NodeInstanceWrapper
  create_node_instance(const rclcpp::NodeOptions & options) override
  {
    // 直接 new 出用户定义的节点，传入 options
    auto node = std::make_shared<NodeT>(options);

    // NodeInstanceWrapper 封装节点，提供统一的接口
    return NodeInstanceWrapper(
      node,
      [](rclcpp::node_interfaces::NodeBaseInterface::SharedPtr) {}
    );
  }
};
```

---

## 三、component_container 入口解析

### 3.1 单线程容器：`component_container.cpp`

```cpp
// rclcpp_components/src/component_container.cpp

int main(int argc, char * argv[])
{
  // 1. 初始化 rclcpp
  rclcpp::init(argc, argv);
  
  // 2. 创建单线程 executor
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  
  // 3. 创建 ComponentManager（自身也是一个节点，加入 executor）
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  
  // 4. 开始事件循环（spin）
  exec->spin();
  
  rclcpp::shutdown();
  return 0;
}
```

### 3.2 多线程容器：`component_container_mt.cpp`

```cpp
// rclcpp_components/src/component_container_mt.cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 多线程执行器，默认使用 std::thread::hardware_concurrency() 个线程
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
  
  rclcpp::shutdown();
  return 0;
}
```

### 3.3 隔离容器：`component_container_isolated.cpp`（Iron+）

```cpp
// 每个组件拥有独立的 SingleThreadedExecutor（完全隔离，但无进程内通信优化）
// ComponentManagerIsolated 为每个加载的节点创建独立线程和 executor
```

---

## 四、class_loader 机制解析

`class_loader` 是 `ComponentManager` 动态加载 `.so` 的底层工具。

### 4.1 工作原理

```
.so 文件 (libmy_component.so)
  │
  ├── 包含 CLASS_LOADER_REGISTER_CLASS 宏展开的静态初始化代码
  │   └── 在 dlopen() 时自动执行，向全局注册表注册工厂函数
  │
  └── dlopen() → class_loader::ClassLoader::loadLibrary()
                    → dlopen(path, RTLD_LAZY | RTLD_GLOBAL)
                    → 触发静态初始化 → 工厂注册完成
```

### 4.2 CLASS_LOADER_REGISTER_CLASS 宏展开

```cpp
// class_loader/include/class_loader/register_macro.hpp
#define CLASS_LOADER_REGISTER_CLASS(Derived, Base)                          \
  CLASS_LOADER_REGISTER_CLASS_WITH_MESSAGE(Derived, Base, "")

// 展开后实质上是：
namespace {
  struct ProxyExecMyPlugin {
    ProxyExecMyPlugin() {
      // 向 ClassLoader 的全局注册表添加工厂函数
      class_loader::impl::registerPlugin<Derived, Base>(
        typeid(Derived).name(),    // 类型 ID 字符串
        "MyPlugin"                 // 用户可读名称
      );
    }
  } g_register_my_plugin;   // 全局对象，程序启动（或 dlopen）时构造
}
```

### 4.3 ament_index 组件资源查询

```cpp
// 组件资源通过 ament_index 在安装时注册：
// install/share/<pkg>/rclcpp_components/
//   └── node_plugin.xml 或 component_plugins.ini

// 查询方法：
#include "ament_index_cpp/get_resource.hpp"

std::string content;
std::string base_path;
ament_index_cpp::get_resource(
  "rclcpp_components",   // resource type
  package_name,          // 包名
  content,               // 输出：plugin 名称 → .so 路径的映射
  &base_path
);
```

---

## 五、用户如何编写 Component

### 5.1 最小化示例

```cpp
// my_talker_component.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace demo_nodes_cpp {

class Talker : public rclcpp::Node
{
public:
  // 必须接受 NodeOptions 参数（Component 加载时使用）
  explicit Talker(const rclcpp::NodeOptions & options)
  : Node("talker", options)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello, World!";
        pub_->publish(std::move(msg));  // ★ 进程内通信时用 std::move
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp

// 注册宏：生成 NodeFactory 具体类，并通过 class_loader 注册
RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Talker)
```

### 5.2 CMakeLists.txt 配置

```cmake
# 将组件编译为共享库
add_library(talker_component SHARED my_talker_component.cpp)
ament_target_dependencies(talker_component rclcpp rclcpp_components std_msgs)

# 注册 Component（生成 ament_index 资源和 plugin.xml）
rclcpp_components_register_nodes(talker_component "demo_nodes_cpp::Talker")

# 安装共享库
install(TARGETS talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

### 5.3 package.xml 配置

```xml
<package format="3">
  <name>my_talker</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 六、进程内通信（Intra-Process Communication）

### 6.1 启用方式

```cpp
// 通过 NodeOptions 启用
rclcpp::NodeOptions options;
options.use_intra_process_comms(true);

// 或在 YAML 参数文件中：
# use_intra_process_comms: true
```

### 6.2 IntraProcessManager 源码解析

```cpp
// rclcpp/include/rclcpp/intra_process_manager.hpp

class IntraProcessManager
{
public:
  // 注册发布者（返回进程内通信 ID）
  uint64_t
  add_publisher(rclcpp::PublisherBase::SharedPtr publisher);

  // 注册订阅者
  uint64_t
  add_subscription(rclcpp::SubscriptionBase::SharedPtr subscription);

  // 核心：发布消息
  // 返回消息是否需要继续走 DDS 路径（进程外订阅者存在时）
  template<typename MessageT, typename Alloc, typename Deleter>
  bool
  do_intra_process_publish(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT, Deleter> & message,
    typename std::allocator_traits<Alloc>::template rebind_alloc<MessageT> & allocator);

private:
  // 主题 → {发布者列表, 订阅者列表} 的映射
  std::unordered_map<std::string, PublisherToSubscriptionIdsMap> pub_to_subs_;
  
  // 订阅者 ID → 消息缓冲区（ring buffer）
  std::unordered_map<uint64_t, SubscriptionWaitSetEntry> subscriptions_;
};
```

### 6.3 进程内通信数据流

```
Publisher::publish(msg)
  │
  ├─ [进程内订阅者存在?]
  │   YES → IntraProcessManager::do_intra_process_publish()
  │            │
  │            ├─ 将 unique_ptr 存入目标订阅者的 ring buffer
  │            │   （零拷贝，只传递指针所有权）
  │            └─ 唤醒订阅者 wait set（通过 rcl_guard_condition）
  │
  └─ [进程外订阅者存在?]
      YES → rcl_publish() → DDS 层序列化发送
```

---

## 七、静态组合（Static Composition）

除运行时动态加载外，还可以在编译时将组件静态链接成单个可执行文件：

```cpp
// manual_composition.cpp（ros2/demos 示例）

#include "composition/talker_component.hpp"
#include "composition/listener_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  
  // 直接实例化，无需 ComponentManager
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  
  auto talker = std::make_shared<composition::Talker>(options);
  auto listener = std::make_shared<composition::Listener>(options);
  
  exec.add_node(talker);
  exec.add_node(listener);
  exec.spin();
  
  rclcpp::shutdown();
  return 0;
}
```

---

## 八、ROS2 服务接口（ComponentManager 提供的三个服务）

```
# composition_interfaces/srv/LoadNode.srv
string package_name
string plugin_name
string node_name        # 可选，覆盖默认名称
string node_namespace   # 可选
composition_interfaces/NodeDescription[] remap_rules
rcl_interfaces/Parameter[] parameters
rcl_interfaces/ParameterValue[] extra_arguments
---
uint64 unique_id
string full_node_name
bool success
string error_message
```

```
# composition_interfaces/srv/UnloadNode.srv
uint64 unique_id
---
bool success
string error_message
```

```
# composition_interfaces/srv/ListNodes.srv
---
string[] full_node_names
uint64[] unique_ids
```

---

## 九、常见问题与源码关联

### Q1: 为什么 Component 构造函数必须接受 `NodeOptions`？

```cpp
// NodeFactoryTemplate<NodeT>::create_node_instance() 中：
auto node = std::make_shared<NodeT>(options);
//                                   ^^^^^^ options 来自 LoadNode 请求
// 如果构造函数不接受 NodeOptions，编译期报错
```

### Q2: 同一 executor 中多个 Component 如何互不阻塞？

通过 `MultiThreadedExecutor` + `MutuallyExclusiveCallbackGroup` 或 `ReentrantCallbackGroup`：
```cpp
// 在组件构造函数内：
auto cb_group = this->create_callback_group(
  rclcpp::CallbackGroupType::MutuallyExclusive);
auto options = rclcpp::SubscriptionOptions();
options.callback_group = cb_group;
sub_ = this->create_subscription<Msg>("topic", 10, callback, options);
```

### Q3: 如何确认进程内通信生效？

```bash
# 查看节点是否使用进程内通信
ros2 topic echo /chatter --no-daemon
# 如果看到消息，但没有通过 DDS 网络（抓包无数据），则进程内通信生效

# 或查看节点参数
ros2 param get /talker use_intra_process_comms
```

---

## 十、源码阅读推荐路径

```
1. rclcpp_components/src/component_manager.cpp   # 主体逻辑
2. rclcpp_components/src/component_container.cpp # 单线程容器入口
3. rclcpp/src/rclcpp/intra_process_manager.cpp   # 进程内通信核心
4. class_loader/src/class_loader.cpp             # 动态加载实现
5. ros2/demos/composition/                       # 官方示例
```

**GitHub 直链**：
- https://github.com/ros2/rclcpp/tree/rolling/rclcpp_components/src
- https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/intra_process_manager.cpp
- https://github.com/ros/class_loader/blob/rolling/src/class_loader.cpp
- https://github.com/ros2/demos/tree/rolling/composition
