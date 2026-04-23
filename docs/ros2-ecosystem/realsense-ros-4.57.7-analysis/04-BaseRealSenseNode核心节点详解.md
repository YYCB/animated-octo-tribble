# RealSense ROS 4.57.7 源码分析 - BaseRealSenseNode 核心节点详解

## 1. 概述

`BaseRealSenseNode` 是整个系统的核心类，负责：
- 传感器枚举与管理
- 数据流的开启/关闭
- 帧数据回调与处理
- ROS 话题/服务/Action 的创建与发布
- TF 变换管理
- 后处理滤波器管道
- IMU 数据融合
- 动态参数管理

### 文件位置
- 头文件：`include/base_realsense_node.h` (21KB)
- 实现分散在 6 个文件中：
  - `src/base_realsense_node.cpp` (55KB) - 核心帧处理、数据发布
  - `src/rs_node_setup.cpp` (30KB) - 初始化、话题创建/销毁
  - `src/parameters.cpp` (6KB) - 参数读取
  - `src/tfs.cpp` (13KB) - TF 坐标变换
  - `src/safety.cpp` (9KB) - 安全相机功能
  - `src/actions.cpp` (6KB) - ROS2 Action 实现

## 2. 构造函数

```cpp
BaseRealSenseNode::BaseRealSenseNode(
    RosNodeBase& node,           // ROS 节点引用（RealSenseNodeFactory自身）
    rs2::device dev,             // librealsense2 设备对象
    std::shared_ptr<Parameters> parameters,  // 参数管理器
    bool use_intra_process = false           // 是否启用进程内通信
);
```

**初始化列表** 设置的默认值：
- `_is_running = true`
- `_depth_scale_meters = 0`
- `_sync_frames = false` (SYNC_FRAMES 常量)
- `_enable_rgbd = false`
- `_imu_sync_method = NONE`
- `_is_profile_changed = false`
- `_safety_sensor = nullptr`

构造时还会调用 `initializeFormatsMaps()` 初始化格式映射表。

## 3. 生命周期 (`publishTopics()`)

```cpp
void BaseRealSenseNode::publishTopics() {
    getParameters();  // 读取所有 ROS 参数
    setup();          // 执行完整的初始化流程
    ROS_INFO_STREAM("RealSense Node Is Up!");
}
```

## 4. 参数获取 (`getParameters()`)

在 `src/parameters.cpp` 中实现，读取以下参数：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `camera_name` | string | "camera" | 相机名称前缀 |
| `base_frame_id` | string | "link" | 基准坐标系 ID |
| `tf_prefix` | string | "" | TF 前缀 |
| `json_file_path` | string | "" | 高级模式 JSON 文件 |
| `clip_distance` | float | -2.0 | 深度裁切距离 |
| `linear_accel_cov` | double | 0.01 | 线性加速度协方差 |
| `angular_velocity_cov` | double | 0.01 | 角速度协方差 |
| `hold_back_imu_for_frames` | bool | false | IMU 等待帧同步 |
| `publish_tf` | bool | true | 是否发布 TF |
| `tf_publish_rate` | double | 0.0 | 动态 TF 发布频率 |
| `diagnostics_period` | double | 0.0 | 诊断发布周期 |
| `unite_imu_method` | int | 0 | IMU 融合方法 |
| `enable_sync` | bool | false | 帧同步模式 |
| `enable_rgbd` | bool | false | RGBD 话题使能 |

## 5. 初始化流程 (`setup()`)

```
setup()
  │
  ├── [GPU] initOpenGLProcessing()     // 条件编译: ACCELERATE_GPU_WITH_GLSL
  │
  ├── setDynamicParams()               // 注册可动态修改的参数
  │
  ├── startDiagnosticsUpdater()        // 启动诊断发布器
  │     └── 创建 diagnostic_updater
  │     └── 启动温度监控线程
  │
  ├── setAvailableSensors()            // 【关键】枚举并封装传感器
  │     ├── dev.query_sensors()
  │     ├── setSafetySensorIfAvailable()
  │     ├── [可选] 加载 JSON 高级配置
  │     └── 遍历传感器，创建 RosSensor
  │
  ├── SetBaseStream()                  // 确定基准流（用于TF和外参）
  │
  ├── setupFilters()                   // 初始化 9+3 个滤波器
  │
  ├── setCallbackFunctions()           // 设置异步帧回调
  │     └── _asyncer.start(frame_callback)
  │
  ├── monitoringProfileChanges()       // 启动 Profile 变更监控线程
  │     └── 等待 _cv_mpc 条件变量
  │     └── 调用 updateSensors()
  │
  ├── updateSensors()                  // 启动/停止传感器流
  │     ├── stopRequiredSensors()
  │     └── startUpdatedSensors()
  │
  ├── publishServices()                // 发布 ROS 服务
  │     ├── ~/hw_reset
  │     ├── ~/device_info
  │     ├── ~/calib_config_read
  │     ├── ~/calib_config_write
  │     └── [安全] 各种安全服务
  │
  └── publishActions()                 // 发布 ROS Action
        └── ~/triggered_calibration
```

## 6. 传感器枚举 (`setAvailableSensors()`)

```cpp
// 遍历设备的所有传感器
for(auto&& sensor : _dev_sensors) {
    const std::string module_name(rs2_to_ros(sensor.get_info(RS2_CAMERA_INFO_NAME)));
    
    if (sensor.is<rs2::depth_sensor>() ||
        sensor.is<rs2::color_sensor>() ||
        sensor.is<rs2::safety_sensor>() ||
        sensor.is<rs2::depth_mapping_sensor>()) {
        // 视频类传感器 → 使用 frame_callback_function
        rosSensor = make_unique<RosSensor>(sensor, ..., frame_callback_function, ...);
    }
    else if (sensor.is<rs2::motion_sensor>()) {
        // 运动类传感器 → 使用 imu_callback_function
        rosSensor = make_unique<RosSensor>(sensor, ..., imu_callback_function, ...);
    }
    _available_ros_sensors.push_back(std::move(rosSensor));
}
```

### 回调函数定义

**视频帧回调链**：
```
sensor → frame_callback_function
           │
           ├── [sync_frames || filters_enabled] → _asyncer.invoke(frame) → frame_callback()
           └── [otherwise] → frame_callback()  直接调用
```

**IMU 回调链**：
```
sensor → imu_callback_function
           ├── imu_callback()            // 独立发布 accel/gyro
           └── imu_callback_sync(frame)  // 融合发布 united IMU
```

## 7. 传感器更新 (`updateSensors()`)

```
updateSensors()
  │
  ├── stopRequiredSensors()
  │     对于每个 available_ros_sensor:
  │       ├── getUpdatedProfiles(wanted_profiles)
  │       ├── 如果 active != wanted:
  │       │     ├── sensor.stop()
  │       │     └── stopPublishers(active_profiles)
  │       └── 如果 wanted 为空:
  │             └── sensor.stop()
  │
  └── startUpdatedSensors()
        对于每个 available_ros_sensor:
          ├── getUpdatedProfiles(wanted_profiles)
          ├── 如果有需要的 Profile:
          │     ├── startPublishers(wanted_profiles, sensor)
          │     ├── sensor.start(wanted_profiles)
          │     ├── updateProfilesStreamCalibData(wanted_profiles)
          │     ├── publishStaticTransforms()
          │     └── startDynamicTf()
          └── startRGBDPublisherIfNeeded()
```

## 8. 帧回调处理 (`frame_callback()`)

这是数据处理的核心，在 `base_realsense_node.cpp` 中实现：

```cpp
void BaseRealSenseNode::frame_callback(rs2::frame frame) {
    // 1. 处理 frameset（同步的多帧）
    if (frame.is<rs2::frameset>()) {
        auto frameset = frame.as<rs2::frameset>();
        
        // 2. 应用滤波器管道
        for (auto& filter : _filters) {
            if (filter->is_enabled()) {
                frameset = filter->process(frameset);
            }
        }
        
        // 3. 遍历 frameset 中的每一帧
        for (auto it = frameset.begin(); it != frameset.end(); ++it) {
            auto f = *it;
            auto stream_type = f.get_profile().stream_type();
            
            // 4. 时间基准设置
            setBaseTime(frame_time, time_domain);
            
            // 5. 按流类型分发处理
            if (stream_type == RS2_STREAM_LABELED_POINT_CLOUD) {
                publishLabeledPointCloud(...);
            } else if (stream_type == RS2_STREAM_OCCUPANCY) {
                publishOccupancyFrame(...);
            } else {
                // 6. 常规视频帧发布
                publishFrame(f, t, sip, _images, _info_publishers, _image_publishers);
                
                // 7. 点云处理
                if (pointcloud_enabled) publishPointCloud(...);
                
                // 8. RGBD 处理
                if (rgbd_enabled) publishRGBD(...);
            }
        }
    }
    // 单帧处理（非同步模式）
    else {
        publishFrame(f, t, sip, ...);
    }
}
```

## 9. 帧发布 (`publishFrame()`)

```
publishFrame(frame, time, stream_index_pair, images, info_publishers, image_publishers)
  │
  ├── 1. 格式转换: rs2::frame → cv::Mat (fillCVMatImageAndReturnStatus)
  │        └── 使用 _rs_format_to_cv_format 映射
  │
  ├── 2. 深度特殊处理:
  │     ├── fix_depth_scale()  → 将深度值转换为标准单位
  │     └── clip_depth()       → 裁切超出范围的深度值
  │
  ├── 3. 创建 ROS Image 消息 (fillROSImageMsgAndReturnStatus)
  │     ├── 设置 header (stamp, frame_id)
  │     ├── 设置 encoding (使用 _rs_format_to_ros_format)
  │     ├── 设置 width, height, step
  │     └── 拷贝图像数据
  │
  ├── 4. 发布 Image (image_publishers[sip].publish)
  │
  ├── 5. 发布 CameraInfo (info_publishers[sip].publish)
  │     └── 包含内参矩阵、畸变系数等
  │
  └── 6. 发布 Metadata (publishMetadata)
        └── 帧的 JSON 元数据
```

## 10. IMU 数据处理

### 10.1 独立 IMU 回调 (`imu_callback()`)

```
imu_callback(frame)
  │
  ├── 识别流类型: GYRO / ACCEL / MOTION
  ├── 设置时间基准
  │
  ├── [MOTION 流] → 读取 combined_motion_data
  │     ├── linear_acceleration (x,y,z)
  │     └── angular_velocity (x,y,z)
  │
  └── [GYRO/ACCEL] → 读取 motion_frame 数据
        └── 发布到对应 IMU 话题
```

### 10.2 同步 IMU 回调 (`imu_callback_sync()`)

两种融合方法：

| 方法 | 说明 |
|------|------|
| `COPY` | 用最新的加速度数据配对每个陀螺仪数据 |
| `LINEAR_INTERPOLATION` | 在两个加速度样本之间对陀螺仪时间点做线性插值 |

```
imu_callback_sync(frame)
  │
  ├── CimuData imu_data(stream_index, vector3d, nanoseconds)
  │
  ├── [COPY 方法]
  │     ├── 保存最新 accel 数据
  │     └── 每次 gyro 到来时，与 accel 配对发布
  │
  └── [LINEAR_INTERPOLATION 方法]
        ├── 维护 _imu_history 队列
        ├── 等待至少 2 个 accel + 中间 gyro
        └── 对每个 gyro 时间点，在两个 accel 之间线性插值
```

## 11. 格式映射表

### RS2 → OpenCV
```cpp
RS2_FORMAT_Y8    → CV_8UC1
RS2_FORMAT_Y16   → CV_16UC1
RS2_FORMAT_Z16   → CV_16UC1
RS2_FORMAT_RGB8  → CV_8UC3
RS2_FORMAT_BGR8  → CV_8UC3
RS2_FORMAT_RGBA8 → CV_8UC4
RS2_FORMAT_BGRA8 → CV_8UC4
RS2_FORMAT_YUYV  → CV_8UC2
RS2_FORMAT_UYVY  → CV_8UC2
RS2_FORMAT_RAW8  → CV_8UC1
RS2_FORMAT_RAW10 → CV_16UC1
RS2_FORMAT_RAW16 → CV_16UC1
```

### RS2 → ROS Image Encoding
```cpp
RS2_FORMAT_Y8    → sensor_msgs::image_encodings::MONO8
RS2_FORMAT_Y16   → MONO16
RS2_FORMAT_Z16   → TYPE_16UC1
RS2_FORMAT_RGB8  → RGB8
RS2_FORMAT_BGR8  → BGR8
RS2_FORMAT_RGBA8 → RGBA8
RS2_FORMAT_BGRA8 → BGRA8
RS2_FORMAT_YUYV  → YUV422_YUY2
RS2_FORMAT_UYVY  → YUV422
RS2_FORMAT_RAW8  → TYPE_8UC1
RS2_FORMAT_RAW10 → TYPE_16UC1
RS2_FORMAT_RAW16 → TYPE_16UC1
```

## 12. 析构流程

```
~BaseRealSenseNode()
  │
  ├── _is_running = false
  ├── 通知所有条件变量 (_cv_tf, _cv_temp, _cv_mpc)
  │
  ├── 等待线程结束:
  │     ├── _tf_t (动态 TF 线程)
  │     ├── _monitoring_t (温度监控线程)
  │     └── _monitoring_pc (Profile 监控线程)
  │
  ├── clearParameters()  // 清除所有参数
  │
  └── 停止所有传感器
        └── sensor->stop() for each sensor
```

## 13. 对 HAL 设计的启示

### 13.1 帧处理管道建议
```
HAL Frame Pipeline:
  1. Raw Frame Acquisition (SDK callback)
  2. Frame Synchronization (optional)
  3. Filter Chain (configurable, plugin-based)
  4. Format Conversion (SDK format → standard format)
  5. Data Output (callback to upper layer)
```

### 13.2 IMU 处理建议
- 支持独立发布和融合发布两种模式
- 融合算法可插拔（COPY, LINEAR_INTERPOLATION, 或自定义）
- 时间同步需要仔细处理（硬件时钟 vs 系统时钟）

### 13.3 需要从 BaseRealSenseNode 中抽离的功能
- ROS 话题创建/发布 → 移到 ROS 适配层
- 参数管理 → 移到通用参数层
- TF 变换 → 移到坐标系管理层
- 保留核心的数据处理逻辑在 HAL 层
