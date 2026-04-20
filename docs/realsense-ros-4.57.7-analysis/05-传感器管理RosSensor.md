# RealSense ROS 4.57.7 源码分析 - 传感器管理 (RosSensor)

## 1. 概述

`RosSensor` 是对 librealsense2 `rs2::sensor` 的 ROS 封装，主要职责：
- 继承 `rs2::sensor`，直接拥有 SDK 传感器能力
- 管理传感器的流配置 (ProfilesManager)
- 管理传感器硬件参数 (SensorParams)
- 处理传感器启停和帧回调
- 提供频率诊断
- 管理自动曝光 ROI

### 文件位置
- 头文件：`include/ros_sensor.h`
- 实现：`src/ros_sensor.cpp` (19KB)

## 2. 类结构

```cpp
class RosSensor : public rs2::sensor {
private:
    rclcpp::Logger _logger;
    
    // 帧回调
    std::function<void(rs2::frame)> _origin_frame_callback;  // 原始回调
    std::function<void(rs2::frame)> _frame_callback;         // 包装后的回调（含诊断计数）
    
    // 参数管理
    SensorParams _params;
    
    // 生命周期函数
    std::function<void()> _update_sensor_func;     // 触发 Profile 变更
    std::function<void()> _hardware_reset_func;    // 触发硬件重置
    
    // Profile 管理器（可能有多个：Video + Motion）
    std::vector<std::shared_ptr<ProfilesManager>> _profile_managers;
    
    // 自动曝光 ROI
    rs2::region_of_interest _auto_exposure_roi;
    
    // 参数名列表（用于清理）
    std::vector<std::string> _parameters_names;
    
    // 诊断
    std::shared_ptr<diagnostic_updater::Updater> _diagnostics_updater;
    std::map<stream_index_pair, FrequencyDiagnostics> _frequency_diagnostics;
    
    bool _force_image_default_qos;
};
```

## 3. 构造流程

```
RosSensor(sensor, parameters, frame_callback, update_func, reset_func, diag, logger, qos, rosbag)
  │
  ├── 继承初始化: rs2::sensor(sensor)
  │
  ├── 包装帧回调:
  │     _frame_callback = [](frame) {
  │         _origin_frame_callback(frame);           // 调用原始回调
  │         _frequency_diagnostics[sip].Tick();      // 更新频率统计
  │     };
  │
  └── setParameters(is_rosbag_file)
        ├── 禁用 HDR（固件限制，需要先禁用再设置参数）
        ├── _params.registerDynamicOptions()  // 注册传感器硬件选项
        ├── UpdateSequenceIdCallback()        // HDR 序列 ID 回调
        └── registerSensorParameters()        // 注册 Profile 参数
```

## 4. 传感器参数注册 (`registerSensorParameters()`)

```cpp
void RosSensor::registerSensorParameters() {
    // 获取传感器支持的所有 Profile
    auto all_profiles = get_stream_profiles();
    auto module_name = rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME));
    
    // 1. 注册视频 Profile 管理器
    auto video_pm = make_shared<VideoProfilesManager>(params, module_name, logger, force_qos);
    video_pm->registerProfileParameters(all_profiles, update_sensor_func);
    if (video_pm->isTypeExist()) {
        _profile_managers.push_back(video_pm);
        registerAutoExposureROIOptions();  // 注册 ROI 参数
    }
    
    // 2. 注册运动 Profile 管理器
    auto motion_pm = make_shared<MotionProfilesManager>(params, logger);
    motion_pm->registerProfileParameters(all_profiles, update_sensor_func);
    if (motion_pm->isTypeExist()) {
        _profile_managers.push_back(motion_pm);
    }
}
```

**关键设计**：一个物理传感器可以同时拥有视频和运动两种 ProfileManager。

## 5. 传感器启动 (`start()`)

```cpp
bool RosSensor::start(const std::vector<stream_profile>& profiles) {
    if (get_active_streams().size() > 0)
        return false;                          // 已在运行
    
    setupErrorCallback();                      // 注册错误通知回调
    rs2::sensor::open(profiles);               // SDK: 打开传感器并配置 Profile
    rs2::sensor::start(_frame_callback);       // SDK: 开始数据流
    
    // 为每个 Profile 创建频率诊断
    for (auto& profile : profiles) {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        _frequency_diagnostics.emplace(sip, 
            FrequencyDiagnostics(STREAM_NAME(sip), profile.fps(), _diagnostics_updater));
    }
    return true;
}
```

## 6. 传感器停止 (`stop()`)

```cpp
void RosSensor::stop() {
    if (get_active_streams().size() == 0)
        return;
    
    _frequency_diagnostics.clear();   // 清除频率诊断
    rs2::sensor::stop();              // SDK: 停止数据流
    close();                          // SDK: 关闭传感器
}
```

## 7. Profile 更新检测 (`getUpdatedProfiles()`)

```cpp
bool RosSensor::getUpdatedProfiles(vector<stream_profile>& wanted_profiles) {
    wanted_profiles.clear();
    auto active_profiles = get_active_streams();  // 当前运行的 Profile
    
    // 从每个 ProfileManager 收集期望的 Profile
    for (auto pm : _profile_managers) {
        pm->addWantedProfiles(wanted_profiles);
    }
    
    // 比较当前 Profile 和期望 Profile
    if (compare_profiles_lists(active_profiles, wanted_profiles)) {
        return false;  // 没有变化
    }
    return true;       // 需要更新
}
```

### Profile 比较算法

```cpp
bool profiles_equal(const stream_profile& a, const stream_profile& b) {
    if (video profile) {
        return (same stream && same index && same width && same height);
    }
    return (same stream && same index);
}

bool compare_profiles_lists(active, wanted) {
    return is_subset(active, wanted) && is_subset(wanted, active);
}
```

## 8. 错误回调 (`setupErrorCallback()`)

```cpp
void RosSensor::setupErrorCallback() {
    set_notifications_callback([&](const rs2::notification& n) {
        // 记录硬件通知
        if (n.get_severity() >= RS2_LOG_SEVERITY_ERROR) {
            ROS_WARN_STREAM("Hardware Notification:" << n.get_description());
        }
        
        // 特定错误触发硬件重置
        vector<string> error_strings = {
            "RT IC2 Config error",
            "Left IC2 Config error"
        };
        if (error matches any error_string) {
            _hardware_reset_func();  // 触发硬件重置
        }
    });
}
```

## 9. 自动曝光 ROI 管理

```
registerAutoExposureROIOptions()
  │
  ├── 获取当前分辨率 (width, height)
  ├── 设置默认 ROI = {0, 0, width-1, height-1}
  │
  └── 注册 4 个 ROS 参数:
        ├── {module}.auto_exposure_roi.left    (min_x)
        ├── {module}.auto_exposure_roi.right   (max_x)
        ├── {module}.auto_exposure_roi.top     (min_y)
        └── {module}.auto_exposure_roi.bottom  (max_y)
        
        每个参数变更 → set_sensor_auto_exposure_roi()
                         └── roi_sensor.set_region_of_interest(roi)
```

## 10. HDR 序列 ID 管理 (`UpdateSequenceIdCallback()`)

HDR (High Dynamic Range) 模式下需要特殊处理：

```
UpdateSequenceIdCallback()
  │
  ├── 检查是否支持 RS2_OPTION_SEQUENCE_ID
  │
  ├── 检查 HDR 是否启用
  │     └── 如果启用了 HDR 且用户设置了 auto_exposure:
  │           └── 警告并禁用 auto_exposure（HDR 不兼容自动曝光）
  │
  ├── [HDR 启用时] 为每个 sequence_id 设置 exposure 和 gain
  │     ├── 临时禁用 HDR
  │     ├── 循环 seq_id = 1..seq_size:
  │     │     ├── 读取 {module}.exposure.{seq_id} 参数
  │     │     ├── 读取 {module}.gain.{seq_id} 参数
  │     │     └── 设置到传感器
  │     └── 恢复 HDR（通过 RAII 析构器）
  │
  └── 注册 sequence_id 参数回调:
        └── 切换 sequence_id 时更新 gain 和 exposure 的 ROS 参数值
```

## 11. QoS 管理

```cpp
rmw_qos_profile_t RosSensor::getQOS(const stream_index_pair& sip) const {
    for (auto& pm : _profile_managers) {
        if (pm->hasSIP(sip)) {
            return pm->getQOS(sip);
        }
    }
    throw std::runtime_error("Given stream has no profile manager");
}
```

## 12. FrequencyDiagnostics 辅助类

```cpp
class FrequencyDiagnostics {
    // 使用 diagnostic_updater::FrequencyStatus 监控帧率
    // 在每次帧到来时 Tick()
    // 自动检测帧率是否在期望范围内
    FrequencyDiagnostics(name, expected_frequency, updater);
    void Tick();  // 每帧调用
};
```

## 13. 对 HAL 设计的启示

### 13.1 传感器抽象接口建议

```
ISensor:
  + getName() → string
  + getSupportedProfiles() → vector<StreamProfile>
  + getActiveProfiles() → vector<StreamProfile>
  + start(profiles, callback) → bool
  + stop()
  + getOption(option) → value
  + setOption(option, value)
  + getOptionRange(option) → OptionRange
  + setNotificationCallback(callback)

IVideoSensor extends ISensor:
  + getIntrinsics(profile) → Intrinsics
  + getExtrinsics(to_profile) → Extrinsics
  + setROI(roi)

IMotionSensor extends ISensor:
  + getIMUInfo() → IMUCalibration
```

### 13.2 关键设计决策
1. **RosSensor 继承 rs2::sensor**：紧耦合 SDK，HAL 应改为组合关系
2. **ProfileManager 作为策略**：好的设计，HAL 可以复用
3. **回调包装模式**：在原始回调外包装诊断逻辑，HAL 可以用装饰器模式
4. **HDR 特殊处理**：展示了设备特定逻辑的复杂性，HAL 需要扩展点
