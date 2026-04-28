# 03 — 传感器与 Profile 管理对比

> **本章回答**:一个设备里有哪些传感器?每个传感器支持什么样的流配置(分辨率/帧率/格式)?用户如何选 Profile?运行时如何切换 Profile?HAL 的 `ISensor` / `IStreamProfile` / `IProfilesManager` 应该怎么设计?

---

## 1. Orbbec 侧:按流索引 map 管理

### 1.1 流键定义

```cpp
using stream_index_pair = std::pair<ob_stream_type, int>;

static const stream_index_pair COLOR       = {OB_STREAM_COLOR,       0};
static const stream_index_pair COLOR_LEFT  = {OB_STREAM_COLOR_LEFT,  0};
static const stream_index_pair COLOR_RIGHT = {OB_STREAM_COLOR_RIGHT, 0};
static const stream_index_pair DEPTH       = {OB_STREAM_DEPTH,       0};
static const stream_index_pair INFRA0      = {OB_STREAM_IR,          0};
static const stream_index_pair INFRA1      = {OB_STREAM_IR_LEFT,     0};
static const stream_index_pair INFRA2      = {OB_STREAM_IR_RIGHT,    0};
static const stream_index_pair GYRO        = {OB_STREAM_GYRO,        0};
static const stream_index_pair ACCEL       = {OB_STREAM_ACCEL,       0};
```

### 1.2 `OBCameraNode` 中的 map 集合

```cpp
std::map<stream_index_pair, std::shared_ptr<ob::Sensor>>        sensors_;
std::map<stream_index_pair, sensor_msgs::msg::CameraInfo>       camera_infos_;
std::map<stream_index_pair, int>                                width_, height_, fps_;
std::map<stream_index_pair, ob_format>                          format_;
std::map<stream_index_pair, bool>                               enable_stream_;
std::map<stream_index_pair, std::shared_ptr<ob::StreamProfile>> stream_profile_;
std::map<stream_index_pair, std::shared_ptr<image_publisher>>   image_publishers_;
std::map<stream_index_pair, bool>                               flip_stream_, mirror_stream_;
std::map<stream_index_pair, int>                                rotation_stream_;
```

**特点**:
- 非常扁平,每种语义一张 map。
- 没有独立的 `Sensor` / `Profile` 封装类,完全依赖原生 `ob::Sensor` / `ob::StreamProfile`。

### 1.3 Profile 设置流程

```cpp
setupProfiles()  // OBCameraNode
  ├── 遍历 sensors_
  ├── 对每个 sensor:
  │    ├── auto profile_list = sensor->getStreamProfileList()
  │    ├── 按用户配置 width/height/fps/format 挑选最匹配的 profile
  │    │    └── 支持 "0" = default,SDK 自动选择
  │    └── stream_profile_[sip] = matched_profile
  │
  └── 通过 ob::Pipeline 应用:
       ├── config_->enableStream(matched_profile)
       └── pipeline_->start(config_, frame_callback)
```

### 1.4 Profile 匹配策略

**精确匹配优先,失败则按优先级回退**:
```
用户 (w, h, fps, format)  →  sensor->getStreamProfileList()
                              │
                              ├── 精确匹配 (w, h, fps, format)
                              ├── 若 fps=0 → 匹配 (w, h, format, any fps)
                              ├── 若 format=默认 → 匹配第一个同分辨率
                              └── 最终失败 → 报错并使用 default
```

### 1.5 Profile 运行时切换

Orbbec 不支持"不停整个 Pipeline 就动态改 Profile"。变更需要:
```
参数变化 → 回调触发 → stopStreams() → 重建 config → pipeline->start()
```

---

## 2. RealSense 侧:两级抽象 `RosSensor` + `ProfilesManager`

### 2.1 `RosSensor` 直接继承 `rs2::sensor`

```cpp
class RosSensor : public rs2::sensor {
private:
    std::function<void(rs2::frame)> _origin_frame_callback;
    std::function<void(rs2::frame)> _frame_callback;    // 包装后的(含诊断)
    SensorParams _params;
    std::function<void()> _update_sensor_func;
    std::function<void()> _hardware_reset_func;
    std::vector<std::shared_ptr<ProfilesManager>> _profile_managers;  // 可能多个
    rs2::region_of_interest _auto_exposure_roi;
    std::map<stream_index_pair, FrequencyDiagnostics> _frequency_diagnostics;
};
```

### 2.2 `ProfilesManager` 层次

```
ProfilesManager (抽象基类)
  ├── VideoProfilesManager  (depth/color/infrared/safety 流)
  └── MotionProfilesManager (gyro/accel/motion 流)
```

**关键设计**:**一个物理 `RosSensor` 可以同时拥有多个 `ProfilesManager`**——例如 Motion Module 既支持 gyro 又支持 accel,两者通过不同的 PM 管理。

### 2.3 `ProfilesManager` 基类接口

```cpp
class ProfilesManager {
protected:
    SensorParams _params;
    std::map<stream_index_pair, std::shared_ptr<bool>> _enabled_profiles;
    std::map<stream_index_pair, std::shared_ptr<std::string>> _profiles_image_qos_str;
    std::map<stream_index_pair, std::shared_ptr<std::string>> _profiles_info_qos_str;
    std::vector<rs2::stream_profile> _all_profiles;

public:
    virtual bool isWantedProfile(const stream_profile&) = 0;
    virtual void registerProfileParameters(vector<stream_profile>, function<void()>) = 0;

    bool isTypeExist();                                   // 这个 sensor 是否有对应类型的流
    void addWantedProfiles(vector<stream_profile>& out);  // 用户期望的流
    bool hasSIP(const stream_index_pair&) const;
    rmw_qos_profile_t getQOS(const stream_index_pair&) const;
};
```

### 2.4 `VideoProfilesManager` 的参数注册

```
registerProfileParameters(all_profiles, update_sensor_func)
  │
  ├── 提取唯一 SIP 集合: {DEPTH,0}, {INFRARED,1}, {INFRARED,2}, {COLOR,0}
  │
  ├── 注册使能参数: enable_depth / enable_color / enable_infra / enable_infra1 / enable_infra2 / enable_safety
  │
  ├── 注册 Profile 参数: {module}.{stream}_profile = "width,height,fps"
  │    └── 也可分解: width/height/fps 独立
  │
  └── 参数变化 → update_sensor_func() 被调 → 触发 RosSensor 重启
```

### 2.5 Profile 更新检测

```cpp
bool RosSensor::getUpdatedProfiles(vector<stream_profile>& wanted) {
    wanted.clear();
    auto active = get_active_streams();

    for (auto pm : _profile_managers)
        pm->addWantedProfiles(wanted);

    if (compare_profiles_lists(active, wanted)) return false;  // 无变化
    return true;  // 需要重启 sensor
}
```

### 2.6 `BaseRealSenseNode::monitoringProfileChanges` 线程

有一个独立线程周期性检查 `_is_profile_changed` 标志,触发时:
```
updateSensors():
  for each sensor:
    if getUpdatedProfiles():
      sensor.stop();
      sensor.start(new_profiles);
```

⚠️ **注意**:RealSense 是"传感器级"停启,不是整机级停启——只重启变化的传感器,比 Orbbec 细粒度得多。

### 2.7 HDR / 序列 Profile

RealSense 的 VideoProfilesManager 还处理 HDR 多曝光序列的选择(通过 `UpdateSequenceIdCallback`),这是 Orbbec 暴露为 `HDRMergeFilter` 的同一能力从另一侧暴露。

### 2.8 支持的像素格式(实际被 Wrapper 处理)

| 名称 | 说明 |
|---|---|
| `Z16` | 16-bit 深度 |
| `Y8` / `Y16` | IR 灰度 |
| `RGB8` / `BGR8` / `RGBA8` / `BGRA8` | 彩色 |
| `YUYV` / `UYVY` | YUV 4:2:2 |
| `RAW8` / `RAW10` / `RAW16` | 原始 Bayer |
| `Y8I` / `Y12I` | 交错双 IR |

---

## 3. 对比总览

| 维度 | Orbbec | RealSense |
|---|---|---|
| 流键 | `stream_index_pair<ob_stream_type,int>` | `stream_index_pair<rs2_stream,int>` |
| Sensor 抽象 | 无,原生 `ob::Sensor` 在 map 中 | **`RosSensor : public rs2::sensor`** |
| Profile 管理 | 散在 `width_/height_/fps_/format_` map | **`ProfilesManager` 层次化封装** |
| 多 PM per Sensor | ❌ | ✅ (Video + Motion) |
| Profile 参数 | 独立 `color_width` / `color_height` / `color_fps` | 组合 `rgb_camera.color_profile = "w,h,fps"` + 独立 |
| QoS 粒度 | 全局 | **每流可单独配 image_qos / info_qos** |
| 运行时切换粒度 | 整个 Pipeline 重启 | **仅重启变化的 Sensor** |
| Profile 匹配 | 精确匹配 + 回退 | 参数化,由 `isWantedProfile()` 返回 true/false |
| 自动选择 | 0/空 → SDK 默认 | 0/空 → SDK 默认 |
| 支持 `is_active_streams()` | 隐式(pipeline 运行即所有流激活) | 显式 `get_active_streams()` |

### ⚠️ 关键差异

1. **重启粒度**:
   - RealSense **按 sensor 重启**,改 depth 的 fps 不影响 color 传感器。
   - Orbbec 改任何 Profile 都要 stop/start 整个 Pipeline。
   - **HAL 决策**:要么 HAL 接口暴露 `ISensor::restart(new_profiles)` 和 `IDevice::restart()` 两级,由后端选择实现粒度;要么统一采用 sensor 级(更优)。

2. **参数组合 vs 分解**:
   - RealSense 同时支持 `rgb_camera.color_profile = "1280,720,30"` 和分解参数 `rgb_camera.color_width = 1280`。
   - Orbbec 只有分解形式。
   - **HAL 决策**:`IStreamProfile` 结构化字段 + ROS Adapter 提供两种参数表达。

3. **多 PM 设计(RealSense 特色)**:
   - 同一个物理 Motion Module 同时 accel + gyro,需要 VideoPM 不合适,MotionPM 专门处理。
   - Orbbec 把 IMU 单独拉了个 Pipeline(参见第 8 章)来规避这个问题。
   - **HAL 决策**:`ISensor` 应返回 `std::vector<StreamProfile>`,同一 Sensor 可以产出多种类型的流。不需要"多 PM" 这种内部结构。

4. **Profile 匹配算法**:
   - 两家都有"用户指定 → SDK 候选列表 → 挑一个"的逻辑。
   - RealSense 通过 `isWantedProfile()` 虚函数由子类实现,清晰解耦。
   - Orbbec 内置在 `setupProfiles` 的循环里。
   - **HAL 决策**:提供 `IProfileMatcher` 接口,默认实现为精确匹配 + 回退。

---

## 4. 对 HAL 的可复用要点

### 4.1 `StreamProfile` 值类型定义

```cpp
enum class StreamType {
    Depth, Color, ColorLeft, ColorRight,
    Infrared, InfraredLeft, InfraredRight,
    Gyro, Accel, Motion,
    Safety, LabeledPointCloud, Occupancy,
    Lidar, Scan,
};

enum class PixelFormat {
    Z16, Y8, Y16, RGB8, BGR8, RGBA8, BGRA8,
    YUYV, UYVY, NV12, NV21,
    RAW8, RAW10, RAW16,
    MJPG, H264, H265,     // 压缩格式 (Orbbec)
    Y8I, Y12I,            // 交错双 IR (RealSense)
    Unknown
};

struct StreamIndex {
    StreamType type;
    int index = 0;
    bool operator==(const StreamIndex&) const;
    bool operator<(const StreamIndex&) const;
};

struct StreamProfile {
    StreamIndex stream;
    int width;
    int height;
    int fps;
    PixelFormat format;

    bool matches(const StreamProfile& other) const {  // 支持 0/unknown 作通配
        return (width == 0 || width == other.width)
            && (height == 0 || height == other.height)
            && (fps == 0 || fps == other.fps)
            && (format == PixelFormat::Unknown || format == other.format)
            && stream == other.stream;
    }
};
```

### 4.2 `ISensor` 接口

```cpp
class ISensor {
public:
    virtual std::string getName() const = 0;          // "Depth Module", "RGB Camera", "Motion Module"
    virtual SensorCategory getCategory() const = 0;   // Video / Motion / Lidar / Safety

    // Profile
    virtual std::vector<StreamProfile> getSupportedProfiles() const = 0;
    virtual std::vector<StreamProfile> getActiveProfiles() const = 0;

    // 启停
    virtual bool start(const std::vector<StreamProfile>& profiles,
                       FrameCallback callback) = 0;
    virtual void stop() = 0;
    virtual bool isStreaming() const = 0;

    // 选项 (见第 5 章)
    virtual std::vector<OptionInfo> getSupportedOptions() const = 0;
    virtual float getOption(const std::string& name) const = 0;
    virtual void setOption(const std::string& name, float value) = 0;

    // ROI (RealSense 风格)
    virtual bool supportsAutoExposureROI() const { return false; }
    virtual void setAutoExposureROI(int x1, int y1, int x2, int y2) {}

    // 通知
    virtual void setNotificationCallback(
        std::function<void(const std::string& desc, int severity)>) = 0;
};
```

### 4.3 可选的 `IProfileMatcher`

```cpp
class IProfileMatcher {
public:
    virtual StreamProfile match(
        const StreamProfile& requested,
        const std::vector<StreamProfile>& supported) = 0;
};

class DefaultProfileMatcher : public IProfileMatcher {
    // 精确 → fps 任意 → format 任意 → 第一个
};
```

### 4.4 Profile 变更的两级能力

```cpp
class IDevice {
    // 通用(整机重启,Orbbec 后端总是实现这个)
    virtual void restart(const StreamConfig& new_config) = 0;

    // 可选(sensor 级,RealSense 后端实现)
    virtual bool supportsPerSensorRestart() const = 0;
};

// ISensor::start/stop 已经覆盖了 sensor 级重启
```

ROS Adapter 层在参数变化时:
```
if (device->supportsPerSensorRestart()):
    只 sensor->stop(); sensor->start(new_profiles);
else:
    device->restart(new_config);
```

### 4.5 QoS 支持

```cpp
struct StreamPublisherConfig {
    StreamIndex stream;
    std::string image_qos;        // "SENSOR_DATA", "DEFAULT", ...
    std::string info_qos;
};
```

HAL 本身不涉及 QoS(那是 ROS 的事),但 ROS Adapter 应按 RealSense 的"每流可单独 QoS" 模式实现,避免一旦有高频流(深度 90fps)拖累低频流。

### 4.6 潜在坑

- ⚠️ **`stream_profile` 是 SDK 持有对象**:两家的 `ob::StreamProfile` / `rs2::stream_profile` 都是 SDK 引用,不能跨设备生命周期保存。HAL 的 `StreamProfile` 必须是纯值类型(POD)。
- ⚠️ **默认值 0 的语义**:两家都用 "0" 表示"由 SDK 决定",HAL 应该显式保留这一语义:`width=0 → 任意`。
- ⚠️ **HDR 模式下的多个 profile**:同一个 sensor 可能启用多个 depth profile(不同曝光),HAL 的 `start` 要接受 `vector<StreamProfile>`。
