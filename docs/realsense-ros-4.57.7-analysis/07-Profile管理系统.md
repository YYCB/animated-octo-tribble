# RealSense ROS 4.57.7 源码分析 - Profile 管理系统

## 1. 概述

Profile 管理系统负责：
- 管理传感器支持的所有流配置（分辨率、帧率、格式）
- 将 Profile 参数暴露为 ROS 动态参数
- 根据用户参数选择合适的 Profile
- 检测 Profile 变更并触发传感器重启

### 文件位置
- 头文件：`include/profile_manager.h`
- 实现：`src/profile_manager.cpp` (28KB)

## 2. 类层次

```
ProfilesManager (抽象基类)
  ├── VideoProfilesManager  (视频流: depth, color, infrared, safety等)
  └── MotionProfilesManager (运动流: gyro, accel, motion)
```

## 3. ProfilesManager 基类

```cpp
class ProfilesManager {
protected:
    rclcpp::Logger _logger;
    SensorParams _params;
    
    // 每个流的使能状态
    map<stream_index_pair, shared_ptr<bool>> _enabled_profiles;
    
    // 每个流的 QoS 配置
    map<stream_index_pair, shared_ptr<string>> _profiles_image_qos_str;
    map<stream_index_pair, shared_ptr<string>> _profiles_info_qos_str;
    
    // 所有可用的 Profile
    vector<rs2::stream_profile> _all_profiles;
    
    // 参数名列表（用于清理）
    vector<string> _parameters_names;

public:
    virtual bool isWantedProfile(const stream_profile& profile) = 0;     // 纯虚
    virtual void registerProfileParameters(vector<stream_profile> all_profiles, 
                                           function<void()> update_func) = 0;  // 纯虚
    
    bool isTypeExist();              // 是否存在该类型的 Profile
    void addWantedProfiles(vector<stream_profile>& wanted);  // 添加期望的 Profile
    void clearParameters();          // 清理参数
    bool hasSIP(const stream_index_pair& sip) const;
    rmw_qos_profile_t getQOS(const stream_index_pair& sip) const;
    rmw_qos_profile_t getInfoQOS(const stream_index_pair& sip) const;
};
```

## 4. VideoProfilesManager

### 4.1 成员变量

```cpp
class VideoProfilesManager : public ProfilesManager {
private:
    string _module_name;                       // 模块名 (如 "depth_module", "rgb_camera")
    map<stream_index_pair, rs2_format> _formats;  // 每个流的格式
    map<rs2_stream, int> _fps;                 // 每个流类型的帧率
    map<rs2_stream, int> _width;               // 每个流类型的宽度
    map<rs2_stream, int> _height;              // 每个流类型的高度
    bool _force_image_default_qos;
};
```

### 4.2 参数注册流程

```
registerProfileParameters(all_profiles, update_sensor_func)
  │
  ├── 1. 保存 all_profiles
  │
  ├── 2. 提取唯一的 stream_index_pair 集合
  │        例如: {DEPTH,0}, {INFRARED,1}, {INFRARED,2}, {COLOR,0}
  │
  ├── 3. 获取默认 Profile (getDefaultProfiles)
  │        └── 对每个 SIP，找到 SDK 默认的分辨率/帧率/格式
  │
  ├── 4. 注册使能参数
  │        ├── enable_depth     (bool, default: from launch)
  │        ├── enable_color     (bool, default: from launch)
  │        ├── enable_infra     (bool, default: from launch)
  │        ├── enable_infra1    (bool, default: from launch)
  │        ├── enable_infra2    (bool, default: from launch)
  │        ├── enable_safety    (bool, default: from launch)
  │        └── ...等
  │
  ├── 5. 注册视频参数 (registerVideoSensorParams)
  │        对每个流类型 (depth, color, infra):
  │        ├── {module}.{stream}_profile → "width,height,fps"
  │        ├── 独立的 width/height/fps 也可以配置
  │        └── 变更任一参数 → 触发 update_sensor_func
  │
  ├── 6. 注册格式参数 (registerVideoSensorProfileFormat)
  │        ├── {module}.{stream}_format → "RGB8", "Z16" 等
  │        └── 变更格式 → 触发 update_sensor_func
  │
  └── 7. 注册 QoS 参数
           ├── {module}.{stream}.image_qos → QoS 策略字符串
           └── {module}.{stream}.info_qos  → QoS 策略字符串
```

### 4.3 Profile 匹配算法

```cpp
bool VideoProfilesManager::isWantedProfile(const stream_profile& profile) {
    stream_index_pair sip(profile.stream_type(), profile.stream_index());
    
    // 1. 检查该流是否启用
    if (!_enabled_profiles[sip] || !*_enabled_profiles[sip])
        return false;
    
    // 2. 获取期望的参数
    int width = _width[profile.stream_type()];
    int height = _height[profile.stream_type()];
    int fps = _fps[profile.stream_type()];
    rs2_format format = _formats[sip];
    
    // 3. 如果用户指定了 0,0,0 → 使用默认 Profile
    if (width == 0 && height == 0 && fps == 0) {
        return isSameProfileValues(profile, default_width, default_height, 
                                    default_fps, format);
    }
    
    // 4. 精确匹配
    return isSameProfileValues(profile, width, height, fps, format);
}
```

### 4.4 Profile 验证和回退

```cpp
stream_profile VideoProfilesManager::validateAndGetSuitableProfile(
    rs2_stream stream_type, stream_profile given_profile) 
{
    // 如果给定的 Profile 有效，直接返回
    if (given_profile) return given_profile;
    
    // 否则，尝试找到替代 Profile:
    // 1. 匹配分辨率和格式，但不限制帧率
    // 2. 匹配格式，但不限制分辨率
    // 3. 使用默认 Profile
    
    ROS_WARN("Couldn't find exact profile, using closest match");
    return suitable_profile;
}
```

## 5. MotionProfilesManager

### 5.1 参数注册

```
registerProfileParameters(all_profiles, update_sensor_func)
  │
  ├── 1. 提取运动流的唯一 SIP
  │        例如: {GYRO,0}, {ACCEL,0}, {MOTION,0}
  │
  ├── 2. 注册使能参数
  │        ├── enable_gyro     (bool)
  │        ├── enable_accel    (bool)
  │        └── enable_motion   (bool)
  │
  ├── 3. 注册帧率参数 (registerFPSParams)
  │        ├── gyro_fps    (int, 从可用值列表中选择)
  │        ├── accel_fps   (int)
  │        └── motion_fps  (int)
  │        变更帧率 → 触发 update_sensor_func
  │
  └── 4. 注册 QoS 参数
           ├── {stream}.image_qos
           └── {stream}.info_qos
```

### 5.2 运动 Profile 匹配

```cpp
bool MotionProfilesManager::isWantedProfile(const stream_profile& profile) {
    stream_index_pair sip(profile.stream_type(), profile.stream_index());
    
    // 1. 检查该流是否启用
    if (!_enabled_profiles[sip] || !*_enabled_profiles[sip])
        return false;
    
    // 2. 匹配帧率
    int wanted_fps = *_fps[sip];
    if (wanted_fps == 0) {
        // fps=0 表示使用默认（最高帧率？或SDK默认）
        return isDefaultProfile(profile);
    }
    return profile.fps() == wanted_fps;
}
```

## 6. Profile 格式字符串

支持的格式字符串到 `rs2_format` 的映射：

```
"RGB8"   → RS2_FORMAT_RGB8
"BGR8"   → RS2_FORMAT_BGR8
"RGBA8"  → RS2_FORMAT_RGBA8
"BGRA8"  → RS2_FORMAT_BGRA8
"Y8"     → RS2_FORMAT_Y8
"Y16"    → RS2_FORMAT_Y16
"Z16"    → RS2_FORMAT_Z16
"YUYV"   → RS2_FORMAT_YUYV
"UYVY"   → RS2_FORMAT_UYVY
"RAW8"   → RS2_FORMAT_RAW8
"RAW10"  → RS2_FORMAT_RAW10
"RAW16"  → RS2_FORMAT_RAW16
"Y8I"    → RS2_FORMAT_Y8I
"Y12I"   → RS2_FORMAT_Y12I
```

## 7. QoS 字符串映射

```cpp
const rmw_qos_profile_t qos_string_to_qos(string str) {
    "SYSTEM_DEFAULT" → rmw_qos_profile_system_default
    "DEFAULT"        → rmw_qos_profile_default
    "PARAMETER_EVENTS" → rmw_qos_profile_parameter_events
    "SERVICES_DEFAULT" → rmw_qos_profile_services_default
    "SENSOR_DATA"    → rmw_qos_profile_sensor_data
    "HID_DEFAULT"    → rmw_qos_profile_sensor_data
}
```

## 8. Profile 参数示例

用户可通过以下参数配置流：

```yaml
# 视频流配置
depth_module:
  depth_profile: "848,480,30"    # width,height,fps
  depth_format: "Z16"
  infra_profile: "848,480,30"
  infra1_format: "Y8"
  infra2_format: "Y8"
  
rgb_camera:
  color_profile: "640,480,30"
  color_format: "RGB8"

# 运动流配置  
gyro_fps: 200
accel_fps: 100

# 使能控制
enable_depth: true
enable_color: true
enable_infra1: false
enable_infra2: false
enable_gyro: false
enable_accel: false

# QoS 配置
depth_module.depth.image_qos: "SYSTEM_DEFAULT"
depth_module.depth.info_qos: "SYSTEM_DEFAULT"
```

## 9. 动态 Profile 变更流程

```
用户修改参数 (如 enable_depth: false)
  │
  ├── 参数回调触发 update_sensor_func
  │
  ├── update_sensor_func 设置 _is_profile_changed = true
  │
  ├── 通知 _cv_mpc 条件变量
  │
  ├── monitoringProfileChanges 线程被唤醒
  │
  └── 调用 updateSensors()
        ├── stopRequiredSensors()
        │     ├── 获取每个传感器的 wanted_profiles
        │     ├── 如果 active != wanted → stop + stopPublishers
        │     └── 如果 wanted 为空 → stop
        └── startUpdatedSensors()
              ├── 获取每个传感器的 wanted_profiles
              ├── startPublishers(wanted)
              ├── sensor.start(wanted)
              └── publishStaticTransforms()
```

## 10. 对 HAL 设计的启示

### 10.1 Profile 管理接口

```
IProfileManager:
  + getSupportedProfiles() → vector<StreamProfile>
  + getDefaultProfile(stream_type) → StreamProfile
  + setDesiredProfile(stream_type, config: ProfileConfig)
  + getActiveProfiles() → vector<StreamProfile>
  + hasProfileChanged() → bool

ProfileConfig:
  + width: int (0 = default)
  + height: int (0 = default)
  + fps: int (0 = default)
  + format: Format (NONE = default)
  + enabled: bool

StreamProfile:
  + streamType: StreamType
  + streamIndex: int
  + width: int
  + height: int
  + fps: int
  + format: Format
```

### 10.2 关键设计决策
1. **分辨率/帧率/格式三元组**：Profile 的核心属性
2. **默认值机制**：`0,0,0` 表示使用 SDK 默认值，简化用户配置
3. **动态变更**：支持运行时修改 Profile 并自动重启传感器
4. **QoS 解耦**：QoS 属于 ROS 层，HAL 不需要关注
