# 固件构建与 CI/CD

## 一、micro_ros_setup 构建工具链

### 1.1 工具链概述

`micro_ros_setup` 是官方推荐的生产级构建方案，基于 colcon + CMake 的完整 ROS 2 工具链：

```bash
# 环境要求
# - ROS 2 Humble（完整安装）
# - Python 3.10+
# - CMake 3.16+
# - 目标平台交叉编译工具链（如 arm-none-eabi-gcc）

# 步骤 1：创建 micro-ROS 工作空间
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws

# 克隆 micro_ros_setup
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# 安装依赖
source /opt/ros/humble/setup.bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# 构建 micro_ros_setup
colcon build
source install/local_setup.bash
```

### 1.2 三步构建流程

```bash
# 步骤 2：创建固件工作空间
# 参数：<build_system> <RTOS> <platform> <transport>
ros2 run micro_ros_setup create_firmware_ws.sh \
    freertos \        # build_system: freertos / zephyr / nuttx
    stm32h7           # platform: 在 toolchain/toolchain_<platform>.cmake 中定义

# 步骤 3：配置固件
# 配置传输方式和其他选项
ros2 run micro_ros_setup configure_firmware.sh \
    stm32h7_eval \    # configuration: 板级配置名称
    --transport serial_usb \  # 传输方式
    --dev /dev/ttyUSB0        # 目标设备（烧录用）

# 步骤 4：构建固件
# 这一步编译 micro-ROS 静态库 + 用户应用代码
ros2 run micro_ros_setup build_firmware.sh
```

### 1.3 平台配置文件结构

```
micro_ros_setup/
├── config/
│   ├── freertos/
│   │   ├── stm32h7/            # STM32H7 FreeRTOS 配置
│   │   │   ├── app.c           # 应用入口（用户代码放置位置）
│   │   │   ├── app-colcon.meta # 编译选项（消息包列表）
│   │   │   └── toolchain.cmake # 交叉编译工具链
│   │   ├── esp32/              # ESP32 FreeRTOS 配置
│   │   └── raspberrypi_pico/
│   ├── zephyr/
│   │   └── nrf5340/
│   └── arduino/
│       └── teensy41/
└── scripts/
    ├── create_firmware_ws.sh
    ├── configure_firmware.sh
    └── build_firmware.sh
```

---

## 二、自定义消息集成

### 2.1 extra_packages 机制

将项目自定义消息（`chassis_msgs` 等）编译进 micro-ROS 静态库：

```bash
# 在固件工作空间中添加自定义消息包
cd ~/micro_ros_ws/firmware/mcu_ws

# 方法 A：git submodule（推荐）
mkdir -p extra_packages
git submodule add https://github.com/YourOrg/chassis_msgs.git \
    extra_packages/chassis_msgs

# 方法 B：本地软链接（开发调试）
ln -s /path/to/your/chassis_msgs extra_packages/chassis_msgs
```

```
chassis_msgs/
├── package.xml          # 必须包含 buildtool_depend: rosidl_default_generators
├── CMakeLists.txt
└── msg/
    ├── MotorStatus.msg
    └── ChassisCommand.msg
```

```
# chassis_msgs/msg/MotorStatus.msg
std_msgs/Header header
float64[] position       # rad
float64[] velocity       # rad/s
float64[] current        # A
uint8[]   error_flags
```

```cmake
# chassis_msgs/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(chassis_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/ChassisCommand.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

```bash
# 重新构建（包含自定义消息）
ros2 run micro_ros_setup build_firmware.sh
```

### 2.2 在 MCU 代码中使用自定义消息

```c
// 包含生成的 C 头文件
#include <chassis_msgs/msg/motor_status.h>
#include <chassis_msgs/msg/chassis_command.h>

// 使用方式与标准消息完全相同
chassis_msgs__msg__MotorStatus motor_status;
chassis_msgs__msg__MotorStatus__init(&motor_status);

// 创建 Publisher
rcl_publisher_t motor_pub;
rclc_publisher_init_default(
    &motor_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(chassis_msgs, msg, MotorStatus),
    "/motor_status");
```

### 2.3 app-colcon.meta 配置

控制哪些消息包被编译进静态库（精简 Flash 占用）：

```python
# firmware/mcu_ws/app-colcon.meta
{
  "names": {
    "rmw_microxrcedds": {
      "cmake-args": [
        "-DRMW_UXRCE_MAX_NODES=1",
        "-DRMW_UXRCE_MAX_PUBLISHERS=10",
        "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
        "-DRMW_UXRCE_MAX_SERVICES=3",
        "-DRMW_UXRCE_MAX_CLIENTS=2",
        "-DRMW_UXRCE_MAX_HISTORY=4",
        "-DRMW_UXRCE_TRANSPORT=serial"
      ]
    },
    "microxrcedds_client": {
      "cmake-args": [
        "-DUCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3",
        "-DUCLIENT_SERIAL_TRANSPORT_MTU=512"
      ]
    }
  }
}
```

---

## 三、Arduino / PlatformIO 路径

### 3.1 micro_ros_arduino 预编译库

```cpp
// Arduino IDE / PlatformIO（platformio.ini）
// 无需完整 ROS 2 环境，直接使用预编译库

// platformio.ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
lib_deps =
    https://github.com/micro-ROS/micro_ros_arduino.git#humble

// 代码示例
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>

void setup() {
    // 配置串口传输（Arduino Serial）
    set_microros_serial_transports(Serial);

    // ... 标准 micro-ROS 初始化 ...
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
```

### 3.2 自定义消息重编译预编译库

当标准库不包含所需消息时，需要重新编译：

```bash
# 克隆预编译库仓库
git clone https://github.com/micro-ROS/micro_ros_arduino.git
cd micro_ros_arduino

# 构建 Docker 环境（推荐，避免本地环境污染）
docker run -it --rm \
    -v $(pwd):/micro_ros_arduino \
    microros/micro_ros_static_library_builder:humble \
    bash -c "
        # 添加自定义消息包
        cp -r /path/to/chassis_msgs ./library_generation/extra_packages/

        # 为 Teensy 4.1 重新构建
        /bin/build.sh linux teensy41
    "

# 编译结果：src/libmicroros/libmicroros.a + 头文件
# 将整个目录作为 Arduino 库使用
```

---

## 四、OTA 固件更新

### 4.1 ESP32 OTA + micro-ROS 重连

```c
// ESP32 OTA 升级流程（不中断 WiFi）
#include "esp_ota_ops.h"
#include "esp_https_ota.h"

// OTA Service Handler（在独立 FreeRTOS 任务中运行）
void ota_task(void *arg) {
    for (;;) {
        // 等待 OTA 触发（通过 micro-ROS Service 或 MQTT）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 暂停 micro-ROS（避免写入冲突）
        // 通过 Supervisor 状态机触发 destroy_entities()

        esp_https_ota_config_t ota_config = {
            .http_config = &http_config,
        };
        esp_err_t ret = esp_https_ota(&ota_config);

        if (ret == ESP_OK) {
            // OTA 成功：重启
            esp_restart();
        } else {
            // OTA 失败：恢复 micro-ROS
            // 通知 Supervisor 重新 create_entities()
        }
    }
}
```

```cpp
// 主机端：触发 OTA 的 Service 客户端
// 发送 OTA 固件 URL 给 MCU
ros2 service call /robot1/trigger_ota \
    std_srvs/srv/SetBool "{data: true}"
```

### 4.2 STM32 DFU 模式

```bash
# STM32 DFU（Device Firmware Upgrade）OTA 方案
# 使用 STM32 内置 USB DFU Bootloader

# 步骤 1：通过 micro-ROS Service 触发 MCU 进入 Bootloader 模式
ros2 service call /joint_mcu/enter_dfu std_srvs/srv/Empty

# 步骤 2：等待设备重新枚举为 DFU 设备
sleep 2

# 步骤 3：使用 dfu-util 烧录新固件
dfu-util -d 0483:df11 \
    -a 0 -s 0x08000000:leave \
    -D joint_controller_v1.2.bin

# 步骤 4：MCU 自动重启，micro-ROS Agent 自动重连
```

---

## 五、CI/CD 集成

### 5.1 GitHub Actions 编译流水线

```yaml
# .github/workflows/micro_ros_firmware.yml
name: micro-ROS Firmware CI

on:
  push:
    paths:
      - 'firmware/**'
      - 'chassis_msgs/**'
  pull_request:
    paths:
      - 'firmware/**'

jobs:
  build-stm32h7:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble

    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y \
            gcc-arm-none-eabi \
            libnewlib-arm-none-eabi \
            python3-pip
          pip3 install colcon-common-extensions

      - name: Setup micro-ROS workspace
        run: |
          source /opt/ros/humble/setup.bash
          mkdir -p /micro_ros_ws/src
          cp -r $GITHUB_WORKSPACE/micro_ros_setup \
              /micro_ros_ws/src/micro_ros_setup
          cd /micro_ros_ws
          colcon build
          source install/local_setup.bash
          ros2 run micro_ros_setup create_firmware_ws.sh \
              freertos stm32h7

      - name: Add custom messages
        run: |
          cp -r $GITHUB_WORKSPACE/chassis_msgs \
              /micro_ros_ws/firmware/mcu_ws/extra_packages/

      - name: Build firmware
        run: |
          cd /micro_ros_ws
          source install/local_setup.bash
          ros2 run micro_ros_setup configure_firmware.sh \
              stm32h7_eval --transport serial_usb
          ros2 run micro_ros_setup build_firmware.sh

      - name: Upload artifact
        uses: actions/upload-artifact@v4
        with:
          name: stm32h7-firmware
          path: /micro_ros_ws/firmware/build/*.bin

  build-esp32:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble

    steps:
      - uses: actions/checkout@v4

      - name: Install ESP-IDF
        run: |
          apt-get install -y git wget flex bison gperf python3 python3-pip cmake ninja-build ccache
          git clone --recursive https://github.com/espressif/esp-idf.git /esp-idf
          cd /esp-idf && ./install.sh esp32s3

      - name: Build ESP32 firmware
        run: |
          source /esp-idf/export.sh
          cd $GITHUB_WORKSPACE/firmware/esp32
          idf.py build

      - name: Upload artifact
        uses: actions/upload-artifact@v4
        with:
          name: esp32s3-firmware
          path: ${{ github.workspace }}/firmware/esp32/build/*.bin
```

### 5.2 QEMU 模拟器测试

```bash
# micro_ros_simulator：在主机 x86 上模拟 micro-ROS Client
# 用于集成测试（无需真实 MCU 硬件）

# 方法 A：micro_ros_simulator 包（仅 Linux，pseudo-terminal 模拟 Serial）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/pts/2 &
ros2 run micro_ros_simulator micro_ros_simulator  # 在 /dev/pts/2 上运行

# 方法 B：直接在 Linux 上运行 micro-ROS（linux/host 平台）
# 这允许在 GitHub Actions 中运行集成测试，无需硬件

# micro_ros_setup 支持 linux host 平台
ros2 run micro_ros_setup create_firmware_ws.sh none linux

# 编译为本地可执行文件
ros2 run micro_ros_setup build_firmware.sh

# 在 CI 中：启动 Agent + Simulator，验证消息流
```

```yaml
# .github/workflows/integration_test.yml
  integration-test:
    runs-on: ubuntu-22.04
    steps:
      - name: Start micro-ROS Agent
        run: |
          source /opt/ros/humble/setup.bash
          # 使用 pseudo-terminal 对
          socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
          sleep 1
          # Agent 连接到 pty1
          ros2 run micro_ros_agent micro_ros_agent serial \
              --dev /dev/pts/2 &
          sleep 2

      - name: Run micro-ROS simulator
        run: |
          # Simulator 连接到 pty2，发布测试数据
          ./firmware/build/micro_ros_simulator &
          sleep 3

      - name: Verify topics
        run: |
          source /opt/ros/humble/setup.bash
          # 验证 MCU 端 topic 是否可见
          timeout 5 ros2 topic echo /joint_states \
              --once --no-arr || exit 1
```

---

## 六、版本兼容性矩阵

### 6.1 ROS 2 与 micro-ROS 版本对应

| ROS 2 版本 | micro-ROS 分支 | rmw_microxrcedds | Micro XRCE-DDS Client | 支持状态 |
|-----------|--------------|-----------------|----------------------|---------|
| Galactic | galactic | 2.0.x | 2.4.x | ⚠️ EOL（2022-11） |
| Humble | humble | 3.0.x | 3.0.x | ✅ LTS（2027-05） |
| Iron | iron | 4.0.x | 3.1.x | ⚠️ EOL（2024-11） |
| Jazzy | jazzy | 4.1.x | 3.2.x | ✅ LTS（2029-05） |

> **建议**：生产项目锁定 **Humble** 分支（LTS 至 2027）。新项目可选 **Jazzy**（最新 LTS）。

### 6.2 RTOS 版本兼容性

| RTOS | 版本 | 支持状态 | 备注 |
|------|------|---------|------|
| FreeRTOS | 10.4.x + | ✅ 全面支持 | ESP-IDF 内置；STM32CubeMX 集成 |
| Zephyr | 3.4+ | ✅ 全面支持 | west 工具链 |
| NuttX | 12.0+ | ✅ 全面支持 | PX4 生态常用 |
| Mbed OS | 6.x | ⚠️ 社区维护 | Mbed OS 已停止主动开发 |
| Arduino (bare-metal) | N/A | ✅ 通过 micro_ros_arduino | 无 RTOS 调度器 |

### 6.3 硬件平台 SDK 版本要求

| 硬件 | SDK 版本 | 编译器 | 备注 |
|------|---------|-------|------|
| STM32（所有系列） | STM32CubeH7 1.9+ | arm-none-eabi-gcc 10+ | CubeMX 6.6+ 生成初始化代码 |
| ESP32 / ESP32-S3 | ESP-IDF 5.0+ | xtensa-esp32-elf-gcc | v4.x 也支持但不推荐 |
| Teensy 4.1 | Teensyduino 1.57+ | arm-none-eabi-gcc | micro_ros_arduino 直接支持 |
| nRF5340 | nRF Connect SDK 2.4+ | arm-zephyr-eabi-gcc | Zephyr 3.4 内置 |
| RPi Pico | pico-sdk 1.5+ | arm-none-eabi-gcc 12+ | FreeRTOS + micro-ROS |
