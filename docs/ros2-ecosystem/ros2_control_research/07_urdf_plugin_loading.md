# URDF `<ros2_control>` 解析与 pluginlib 插件加载

> 源码路径：  
> - URDF 解析：`hardware_interface/src/component_parser.cpp`  
> - ResourceManager 加载：`hardware_interface/src/resource_manager.cpp`  
> - pluginlib：`pluginlib/include/pluginlib/class_loader.hpp`

---

## 一、URDF 解析流程（`component_parser.cpp`）

### 1.1 入口函数

```cpp
// hardware_interface/src/component_parser.cpp
std::vector<HardwareInfo>
hardware_interface::parse_control_resources_from_urdf(const std::string & urdf_string)
{
  // Step 1: 用 TinyXML2 解析 URDF
  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_string.c_str());

  // Step 2: 找到 <robot> 根节点
  auto * robot_node = doc.FirstChildElement("robot");

  // Step 3: 遍历所有 <ros2_control> 标签
  std::vector<HardwareInfo> result;
  for (auto * element = robot_node->FirstChildElement("ros2_control");
       element != nullptr;
       element = element->NextSiblingElement("ros2_control"))
  {
    result.push_back(parse_resource_from_data_node(element, urdf_string));
  }
  return result;
}
```

### 1.2 `parse_resource_from_data_node()` 核心逻辑

```cpp
HardwareInfo parse_resource_from_data_node(
  tinyxml2::XMLElement * ros2_control_element,
  const std::string & urdf_string)
{
  HardwareInfo hardware;

  // 读取属性：name, type
  hardware.name = ros2_control_element->Attribute("name");  // 必须唯一
  hardware.type = ros2_control_element->Attribute("type");  // system/actuator/sensor

  // 解析 <hardware> 子节点
  auto * hardware_node = ros2_control_element->FirstChildElement("hardware");
  hardware.hardware_plugin_name =
    hardware_node->FirstChildElement("plugin")->GetText();
  // 例如："my_robot_hardware/MyRobotSystem"

  // 解析 <hardware>/<param>
  for (auto * param = hardware_node->FirstChildElement("param");
       param != nullptr;
       param = param->NextSiblingElement("param"))
  {
    hardware.hardware_parameters[param->Attribute("name")] = param->GetText();
  }

  // 解析 <joint> 列表
  for (auto * joint = ros2_control_element->FirstChildElement("joint");
       joint != nullptr;
       joint = joint->NextSiblingElement("joint"))
  {
    hardware.joints.push_back(parse_component_from_node(joint));
  }

  // 解析 <sensor> 列表
  for (auto * sensor = ros2_control_element->FirstChildElement("sensor"); ...)
  hardware.sensors.push_back(parse_component_from_node(sensor));

  // 解析 <gpio> 列表
  for (auto * gpio = ros2_control_element->FirstChildElement("gpio"); ...)
  hardware.gpios.push_back(parse_component_from_node(gpio));

  // 保存原始 XML 片段（用于 mock_components 等场景）
  hardware.original_xml = serialize_element(ros2_control_element);

  return hardware;
}
```

### 1.3 `parse_component_from_node()` — 解析单个关节/传感器

```cpp
ComponentInfo parse_component_from_node(tinyxml2::XMLElement * component_node) {
  ComponentInfo component;
  component.name = component_node->Attribute("name");

  // 解析 <command_interface>
  for (auto * ci = component_node->FirstChildElement("command_interface"); ...) {
    InterfaceInfo iface;
    iface.name = ci->Attribute("name");   // "position" / "velocity" / "effort" / custom
    // 解析可选的 <param name="min"> / <param name="max"> / <param name="initial_value">
    for (auto * param = ci->FirstChildElement("param"); ...) {
      if (string(param->Attribute("name")) == "min") iface.min = param->GetText();
      if (string(param->Attribute("name")) == "max") iface.max = param->GetText();
      if (string(param->Attribute("name")) == "initial_value") iface.initial_value = param->GetText();
    }
    component.command_interfaces.push_back(iface);
  }

  // 解析 <state_interface>（同上）
  for (auto * si = component_node->FirstChildElement("state_interface"); ...)
    component.state_interfaces.push_back(parse_interface_from_node(si));

  return component;
}
```

---

## 二、`robot_description` 的获取方式

CM 从以下优先级获取 URDF：
1. **参数 `robot_description`**：由 `robot_state_publisher` 发布到 `/robot_description` topic，CM 订阅并存入参数
2. **直接参数设置**：`controller_manager.robot_description` 参数

标准 launch 链：

```python
# launch/robot.launch.py
# 1. robot_state_publisher 读取 URDF 文件，发布 /robot_description + TF
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": xacro_output}]
)

# 2. controller_manager 订阅 /robot_description
controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        {"robot_description": xacro_output},  # 直接传，或从 topic 订阅
        "config/controller_config.yaml"
    ]
)
```

---

## 三、pluginlib 加载机制

### 3.1 pluginlib 原理

`pluginlib` 是 ROS 生态的动态插件系统，基于 C++ 动态库 + XML 注册文件：

```
plugin_name（字符串）
    → ClassLoader 查找 XML 注册表
    → 找到对应 .so 共享库
    → dlopen() 加载 + 调用 createSharedInstance()
    → 返回 shared_ptr<BaseClass>
```

### 3.2 硬件插件注册（以 `my_robot_hardware` 为例）

**步骤 1**：实现类并添加导出宏

```cpp
// src/my_robot_system.cpp
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/system_interface.hpp"

class MyRobotSystem : public hardware_interface::SystemInterface {
  // ... 实现 ...
};

// 关键：导出宏（插件名, 基类名）
PLUGINLIB_EXPORT_CLASS(my_robot_hardware::MyRobotSystem,
                       hardware_interface::SystemInterface)
```

**步骤 2**：创建 XML 注册文件

```xml
<!-- my_robot_hardware_plugin.xml -->
<library path="my_robot_hardware">
  <class
    name="my_robot_hardware/MyRobotSystem"
    type="my_robot_hardware::MyRobotSystem"
    base_class_type="hardware_interface::SystemInterface">
    <description>My Robot System Hardware Interface</description>
  </class>
</library>
```

**步骤 3**：在 `CMakeLists.txt` 中注册

```cmake
# CMakeLists.txt
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(my_robot_hardware SHARED src/my_robot_system.cpp)
target_link_libraries(my_robot_hardware hardware_interface::hardware_interface)

# 关键：将 XML 注册文件安装到 ament 索引
pluginlib_export_plugin_description_file(
  hardware_interface my_robot_hardware_plugin.xml)

install(TARGETS my_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES my_robot_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME})
```

**步骤 4**：`package.xml` 声明依赖

```xml
<package>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <export>
    <hardware_interface plugin="${prefix}/my_robot_hardware_plugin.xml"/>
  </export>
</package>
```

### 3.3 ResourceManager 加载硬件插件

```cpp
// hardware_interface/src/resource_manager.cpp
void ResourceManager::load_hardware_components(
  const std::vector<HardwareInfo> & hardware_info)
{
  // 创建三个 ClassLoader，分别对应三种接口类型
  auto system_loader =
    pluginlib::ClassLoader<SystemInterface>("hardware_interface", "hardware_interface::SystemInterface");
  auto actuator_loader =
    pluginlib::ClassLoader<ActuatorInterface>("hardware_interface", "hardware_interface::ActuatorInterface");
  auto sensor_loader =
    pluginlib::ClassLoader<SensorInterface>("hardware_interface", "hardware_interface::SensorInterface");

  for (const auto & hw_info : hardware_info) {
    std::shared_ptr<void> hw_instance;

    if (hw_info.type == "system") {
      auto hw = system_loader.createSharedInstance(hw_info.hardware_plugin_name);
      // 例如 "my_robot_hardware/MyRobotSystem"
      hw->on_init(hw_info);
      // 触发 on_configure()
      hw->on_configure(State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, ""));
      // 收集接口
      auto state_ifaces = hw->export_state_interfaces();
      auto cmd_ifaces   = hw->export_command_interfaces();
      // 存入 resource_map_
      store_hardware_component(hw_info.name, hw, state_ifaces, cmd_ifaces);
    } else if (hw_info.type == "actuator") { ... }
      else if (hw_info.type == "sensor") { ... }
  }
}
```

### 3.4 控制器插件注册（同样通过 pluginlib）

控制器的注册方式与硬件插件完全相同，基类变为：

```cpp
PLUGINLIB_EXPORT_CLASS(my_controllers::MyController,
                       controller_interface::ControllerInterface)
```

CM 使用独立的 ClassLoader：
```cpp
controller_loader_ =
  pluginlib::ClassLoader<controller_interface::ControllerInterfaceBase>(
    "controller_interface", "controller_interface::ControllerInterfaceBase");
```

---

## 四、Xacro 宏：复用 `<ros2_control>` 片段

真实项目中 URDF 通常用 Xacro 编写：

```xml
<!-- urdf/my_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find my_robot_description)/urdf/ros2_control.xacro"/>
  
  <!-- 引入几何/惯量定义 -->
  <xacro:include filename="links.xacro"/>
  <xacro:include filename="joints.xacro"/>
  
  <!-- 实例化 ros2_control 宏 -->
  <xacro:my_robot_ros2_control
    name="MyRobot"
    use_mock_hardware="$(arg use_mock_hardware)"
    mock_sensor_commands="$(arg mock_sensor_commands)"/>
</robot>
```

```xml
<!-- urdf/ros2_control.xacro -->
<xacro:macro name="my_robot_ros2_control"
             params="name use_mock_hardware:=false mock_sensor_commands:=false">
  <ros2_control name="${name}" type="system">
    <hardware>
      <xacro:if value="${use_mock_hardware}">
        <plugin>mock_components/GenericSystem</plugin>
        <param name="fake_sensor_commands">${mock_sensor_commands}</param>
      </xacro:if>
      <xacro:unless value="${use_mock_hardware}">
        <plugin>my_robot_hardware/MyRobotSystem</plugin>
        <param name="can_interface">can0</param>
      </xacro:unless>
    </hardware>
    
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <!-- ... -->
  </ros2_control>
</xacro:macro>
```

Launch 文件中处理 Xacro：

```python
# launch/robot.launch.py
import xacro

def generate_launch_description():
    xacro_file = os.path.join(pkg_share, "urdf", "my_robot.urdf.xacro")
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={
            "use_mock_hardware": use_mock_hardware,
            "mock_sensor_commands": mock_sensor_commands
        }
    ).toxml()
    
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}]
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_content},
                controller_params_file
            ]
        ),
        # 控制器 spawner
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"]
        ),
    ])
```

---

## 五、调试插件加载问题

常见问题：

| 错误信息 | 原因 | 排查方法 |
|---------|------|---------|
| `Could not load class ...` | 插件未安装或 XML 未注册 | `ros2 control list_hardware_interface_types` |
| `Symbol lookup error` | 依赖库缺失 | `ldd libmy_robot_hardware.so` |
| `Failed to load hardware component` | `on_init()` 返回 ERROR | 检查 URDF 参数是否完整 |
| `Interface ... already claimed` | 多控制器冲突 | `ros2 control list_hardware_interfaces` 查看 claimed 状态 |

诊断命令：
```bash
# 列出所有已注册的硬件插件
ros2 control list_hardware_interface_types
ros2 control list_controller_types

# 查看接口 claim 状态
ros2 control list_hardware_interfaces  # 列出所有接口及其 available/claimed 状态
```
