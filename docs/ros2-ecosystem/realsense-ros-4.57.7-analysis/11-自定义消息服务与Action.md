# RealSense ROS 4.57.7 源码分析 - 自定义消息、服务与 Action

## 1. 概述

`realsense2_camera_msgs` 包定义了 4 个消息、10 个服务和 1 个 Action。

## 2. 自定义消息 (msg)

### 2.1 Extrinsics.msg

```
# 跨流外参：描述不同传感器之间的空间关系
float64[9] rotation      # 列主序 3x3 旋转矩阵
float64[3] translation   # 三维平移向量（米）
```

**用途**：发布在 `~/extrinsics/{base}_to_{stream}` 话题上，提供传感器间的外参信息。

### 2.2 IMUInfo.msg

```
# header.frame_id 设为 "imu_accel" 或 "imu_gyro" 以区分
std_msgs/Header header
float64[12] data             # 校准数据（3x4 矩阵）
float64[3] noise_variances   # 噪声方差
float64[3] bias_variances    # 偏置方差
```

**用途**：发布在 `~/{accel|gyro}/imu_info` 话题上，提供 IMU 校准信息。Latched QoS，节点生命周期内只发布一次。

### 2.3 Metadata.msg

```
std_msgs/Header header
string json_data    # JSON 格式的帧元数据
```

**用途**：发布在 `~/{stream}/metadata` 话题上，包含帧级别的元数据（曝光时间、增益、帧号等）。

### 2.4 RGBD.msg

```
# RGB-D 同步消息
std_msgs/Header header
sensor_msgs/CameraInfo rgb_camera_info    # 彩色相机内参
sensor_msgs/CameraInfo depth_camera_info  # 深度相机内参
sensor_msgs/Image rgb                     # 彩色图像
sensor_msgs/Image depth                   # 深度图像（已对齐到彩色）
```

**用途**：发布在 `~/rgbd` 话题上，提供时间同步的 RGB-D 数据。需要同时启用 `sync_frames`、`align_depth`、Color 和 Depth 流。

## 3. 自定义服务 (srv)

### 3.1 DeviceInfo.srv

```
---
string device_name              # 设备名称 (如 "Intel RealSense D435")
string serial_number            # 序列号
string firmware_version         # 固件版本
string usb_type_descriptor      # USB 类型 (如 "3.2")
string firmware_update_id       # 固件更新 ID
string sensors                  # 传感器列表
string physical_port            # 物理端口
```

**端点**：`~/device_info`
**用途**：查询设备的详细信息。

### 3.2 CalibConfigRead.srv

```
---
string calib_config    # 校准配置 JSON
bool success
string error_message
```

**端点**：`~/calib_config_read`
**用途**：读取设备的校准配置。

### 3.3 CalibConfigWrite.srv

```
string calib_config    # 要写入的校准配置 JSON
---
bool success
string error_message
```

**端点**：`~/calib_config_write`
**用途**：写入校准配置到设备。

### 3.4 SafetyPresetRead.srv

```
int32 index           # 预设索引
---
string safety_preset  # 安全预设内容
bool success
string error_message
```

**端点**：`~/safety_preset_read`
**用途**：读取安全相机预设。

### 3.5 SafetyPresetWrite.srv

```
int32 index            # 预设索引
string safety_preset   # 预设内容
---
bool success
string error_message
```

**端点**：`~/safety_preset_write`
**用途**：写入安全相机预设。

### 3.6 SafetyInterfaceConfigRead.srv

```
int32 calib_location   # 校准位置
---
string safety_interface_config  # 安全接口配置
bool success
string error_message
```

**端点**：`~/safety_interface_config_read`

### 3.7 SafetyInterfaceConfigWrite.srv

```
string safety_interface_config
---
bool success
string error_message
```

**端点**：`~/safety_interface_config_write`

### 3.8 ApplicationConfigRead.srv

```
---
string application_config
bool success
string error_message
```

**端点**：`~/application_config_read`

### 3.9 ApplicationConfigWrite.srv

```
string application_config
---
bool success
string error_message
```

**端点**：`~/application_config_write`

### 3.10 HardwareMonitorCommandSend.srv

```
uint32 opcode       # 操作码
uint32 param1       # 参数1
uint32 param2       # 参数2
uint32 param3       # 参数3
uint32 param4       # 参数4
uint8[] data        # 附加数据
---
bool success
uint8[] result      # 返回数据
string error_message
```

**端点**：`~/hardware_monitor_command_send`
**用途**：发送底层硬件监控命令（通过 debug_protocol）。

## 4. 自定义 Action

### 4.1 TriggeredCalibration.action

```
# request
string json "calib run"     # 校准命令 JSON
---
# result
bool success                # 是否成功
string error_msg            # 错误消息
string calibration          # 校准结果 JSON
float32 health              # 校准健康度
---
# feedback
float32 progress            # 进度 (0-100)
```

**端点**：`~/triggered_calibration`
**用途**：触发式在线校准 (On-Chip Calibration)。

#### Action 执行流程

```
客户端发送 Goal (json = "calib run")
  │
  ├── TriggeredCalibrationHandleGoal()
  │     └── 接受目标 (ACCEPT_AND_EXECUTE)
  │
  ├── TriggeredCalibrationHandleAccepted()
  │     └── 创建新线程执行校准
  │
  └── TriggeredCalibrationExecute() [新线程]
        │
        ├── ac_dev = dev.as<auto_calibrated_device>()
        │
        ├── ac_dev.run_on_chip_calibration(
        │     json,
        │     &health,
        │     progress_callback,  // 发布 feedback
        │     timeout_ms = 120000 // 2分钟超时
        │   )
        │
        ├── [成功] progress == 100%
        │     ├── 提取校准表 (ans[3:])
        │     ├── 转为 JSON 字符串
        │     └── goal_handle->succeed(result)
        │
        ├── [取消] goal_handle->is_canceling()
        │     ├── 发送 "calib abort" 命令
        │     └── goal_handle->canceled(result)
        │
        └── [异常]
              └── goal_handle->abort(result)
```

## 5. 标准 ROS 服务

除了自定义服务，还使用了标准服务：

### 5.1 硬件重置

```
~/hw_reset  (std_srvs/srv/Empty)
```

调用 `_dev.hardware_reset()` 重置设备。

## 6. 完整的话题列表

### 6.1 视频流话题

| 话题 | 消息类型 | 条件 |
|------|---------|------|
| `~/color/image_raw` | sensor_msgs/Image | enable_color=true |
| `~/color/camera_info` | sensor_msgs/CameraInfo | enable_color=true |
| `~/depth/image_rect_raw` | sensor_msgs/Image | enable_depth=true |
| `~/depth/camera_info` | sensor_msgs/CameraInfo | enable_depth=true |
| `~/infra/image_raw` | sensor_msgs/Image | enable_infra=true |
| `~/infra1/image_rect_raw` | sensor_msgs/Image | enable_infra1=true |
| `~/infra2/image_rect_raw` | sensor_msgs/Image | enable_infra2=true |
| `~/aligned_depth_to_color/image_raw` | sensor_msgs/Image | align_depth=true |
| `~/aligned_depth_to_color/camera_info` | sensor_msgs/CameraInfo | align_depth=true |

### 6.2 IMU 话题

| 话题 | 消息类型 | 条件 |
|------|---------|------|
| `~/accel/sample` | sensor_msgs/Imu | enable_accel=true |
| `~/gyro/sample` | sensor_msgs/Imu | enable_gyro=true |
| `~/imu` | sensor_msgs/Imu | unite_imu > 0 |
| `~/accel/imu_info` | IMUInfo | enable_accel=true |
| `~/gyro/imu_info` | IMUInfo | enable_gyro=true |

### 6.3 特殊话题

| 话题 | 消息类型 | 条件 |
|------|---------|------|
| `~/depth/color/points` | sensor_msgs/PointCloud2 | pointcloud.enable=true |
| `~/rgbd` | RGBD | enable_rgbd=true |
| `~/labeled_point_cloud/points` | sensor_msgs/PointCloud2 | enable_labeled_point_cloud=true |
| `~/occupancy` | nav_msgs/GridCells | enable_occupancy=true |

### 6.4 元数据/外参话题

| 话题 | 消息类型 |
|------|---------|
| `~/{stream}/metadata` | Metadata |
| `~/extrinsics/{base}_to_{stream}` | Extrinsics |

## 7. 对 HAL 设计的启示

### 7.1 服务抽象

```
IDeviceService:
  + getDeviceInfo() → DeviceInfo
  + hardwareReset()
  + readCalibConfig() → string
  + writeCalibConfig(config: string)
  + runCalibration(config, progress_callback) → CalibrationResult

ISafetyService:
  + readSafetyPreset(index) → string
  + writeSafetyPreset(index, preset)
  + readSafetyInterfaceConfig(location) → string
  + writeSafetyInterfaceConfig(config)
  + readApplicationConfig() → string
  + writeApplicationConfig(config)
  + sendHardwareCommand(opcode, params, data) → result
```

### 7.2 数据输出抽象

```
IDataOutput:
  + onImage(stream, image, camera_info, metadata)
  + onIMU(stream, imu_data)
  + onPointCloud(points)
  + onRGBD(rgb, depth, rgb_info, depth_info)
  + onExtrinsics(from, to, extrinsics)
```
