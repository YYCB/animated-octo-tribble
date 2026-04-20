# OrbbecSDK v2 接口封装层分析

## 1. SDK 架构概述

OrbbecSDK v2（libobsensor）提供 C 和 C++ 两套 API。ROS2 驱动使用 C++ API。

### 1.1 SDK 核心类层次

```
ob::Context
    │── ob::DeviceList
    │── ob::Device
    │   ├── ob::DeviceInfo
    │   ├── ob::Sensor
    │   └── 属性控制（set/get IntProperty / FloatProperty / BoolProperty）
    │── ob::Pipeline
    │   ├── ob::Config
    │   ├── ob::FrameSet
    │   └── ob::StreamProfile
    │── ob::Frame（基类）
    │   ├── ob::ColorFrame
    │   ├── ob::DepthFrame
    │   ├── ob::IRFrame
    │   ├── ob::AccelFrame
    │   ├── ob::GyroFrame
    │   └── ob::PointsFrame
    └── ob::Filter（基类）
        ├── ob::FormatConvertFilter
        ├── ob::Align
        ├── ob::PointCloudFilter
        ├── ob::DecimationFilter
        ├── ob::SpatialFilter
        ├── ob::TemporalFilter
        ├── ob::HoleFillingFilter
        ├── ob::NoiseRemovalFilter
        ├── ob::ThresholdFilter
        ├── ob::HDRMerge
        └── ob::SequenceIdFilter
```

---

## 2. Context 使用分析

```cpp
// 创建 Context
ctx_ = std::make_unique<ob::Context>(config_path);

// 设置日志级别
ctx_->setLoggerSeverity(OB_LOG_SEVERITY_ERROR);

// 注册设备变更回调
ctx_->registerDeviceChangedCallback(callback);

// 查询设备列表
auto device_list = ctx_->queryDeviceList();

// 创建网络设备
ctx_->createNetDevice(ip, port);
```

---

## 3. Device 属性操作

### 3.1 属性操作 API

```cpp
// 整数属性
device->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, value);
int val = device->getIntProperty(OB_PROP_COLOR_EXPOSURE_INT);

// 布尔属性
device->setBoolProperty(OB_PROP_LASER_BOOL, true);
bool val = device->getBoolProperty(OB_PROP_LASER_BOOL);

// 属性范围
auto range = device->getIntPropertyRange(OB_PROP_COLOR_EXPOSURE_INT);
// range.min, range.max, range.step, range.def
```

### 3.2 ROS2 驱动中使用的属性 ID 对照

| SDK 属性 ID | 说明 | 对应 Service |
|-----------|------|------------|
| OB_PROP_COLOR_EXPOSURE_INT | 彩色曝光 | set/get_color_exposure |
| OB_PROP_COLOR_GAIN_INT | 彩色增益 | set/get_color_gain |
| OB_PROP_COLOR_AUTO_EXPOSURE_BOOL | 彩色自动曝光 | set_color_auto_exposure |
| OB_PROP_COLOR_WHITE_BALANCE_INT | 白平衡 | set/get_white_balance |
| OB_PROP_DEPTH_EXPOSURE_INT | 深度曝光 | set/get_depth_exposure |
| OB_PROP_DEPTH_GAIN_INT | 深度增益 | set/get_depth_gain |
| OB_PROP_IR_EXPOSURE_INT | IR 曝光 | set/get_ir_exposure |
| OB_PROP_LASER_BOOL | 激光使能 | set_laser_enable |
| OB_PROP_LDP_BOOL | LDP 使能 | set_ldp_enable |
| OB_PROP_LASER_ENERGY_LEVEL_INT | 激光能量 | 参数配置 |
| OB_PROP_FAN_WORK_MODE_INT | 风扇模式 | set_fan_work_mode |
| OB_PROP_FLOOD_BOOL | 泛光灯使能 | set_floor_enable |

---

## 4. Pipeline 使用模式

```cpp
// 创建 Pipeline
pipeline_ = std::make_unique<ob::Pipeline>(device);
auto config = std::make_shared<ob::Config>();

// 启用流
config->enableStream(color_profile);
config->enableStream(depth_profile);

// 帧聚合模式
config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE);

// 帧同步（对齐 depth 和 color 时间戳）
config->enableFrameSync();

// 深度对齐
config->setAlignMode(ALIGN_D2C_HW_MODE);

// 启动 Pipeline
pipeline_->start(config, [this](std::shared_ptr<ob::FrameSet> fs) {
    onNewFrameSetCallback(fs);
});

// 停止 Pipeline
pipeline_->stop();
```

---

## 5. StreamProfile 匹配

```cpp
auto profiles = sensor->getStreamProfileList();
for (int i = 0; i < profiles->count(); i++) {
    auto p = profiles->getProfile(i)->as<ob::VideoStreamProfile>();
    if (p->width() == w && p->height() == h &&
        p->fps() == fps && p->format() == fmt) {
        return p;
    }
}
// width/height/fps 为 0 → 使用设备默认值
// format 为 "ANY" → 接受任何格式
```

---

## 6. Filter 使用模式

```cpp
ob::SpatialFilter spatial_filter;
spatial_filter.setAlpha(0.5f);
spatial_filter.setDiffThreshold(50);
auto processed = spatial_filter.process(depth_frame);
```

---

## 7. 错误处理模式

```cpp
// SDK 异常类：ob::Error
try {
    device->setIntProperty(prop, value);
} catch (const ob::Error& e) {
    RCLCPP_ERROR(logger_, "SDK Error: %s", e.getMessage());
    // e.getExceptionType() 获取异常类型
}

// 宏封装（ROS2 驱动内部使用）
TRY_EXECUTE_BLOCK({ device->someOperation(); });
TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_XXX, value);
```

---

## 8. HAL 与 SDK 对照表

| HAL 接口 | Orbbec SDK 类 | RealSense SDK 类 |
|---------|-------------|----------------|
| IDeviceManager | ob::Context | rs2::context |
| IDevice | ob::Device | rs2::device |
| ISensor | ob::Sensor | rs2::sensor |
| IFrame | ob::Frame | rs2::frame |
| IFrameSet | ob::FrameSet | rs2::frameset |
| IFilter | ob::Filter | rs2::filter |
| IStreamProfile | ob::StreamProfile | rs2::stream_profile |
