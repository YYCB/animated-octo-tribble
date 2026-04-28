# 15 — HAL 核心接口详细设计

> **本章目的**:把前 14 章得出的结论汇总为一组**可编译的 C++ 头文件草案**。只定义接口,不给实现;与 ROS 解耦;可直接用于详细设计评审。

> **依赖假设**:C++17,`<string>`、`<vector>`、`<map>`、`<memory>`、`<functional>`、`<chrono>`、`<cstdint>`。Optional: `Eigen3`(矩阵)。严禁暴露 `ob::*` / `rs2::*` / `rclcpp::*`。

---

## 0. 目录

1. 通用类型(Stream / PixelFormat / TimestampDomain)
2. `IFrame` / `IVideoFrame` / `IMotionFrame` / `IPoseFrame` / `IFrameSet`
3. `IOption` & `OptionInfo`
4. `ISensor` / `ISafetySensor`
5. `IStreamProfile`(值类型,非接口)
6. `IDevice`
7. `ICalibration`
8. `ISyncManager`
9. `IDeviceManager`
10. `IFilter` / `IFilterPipeline` / `PointCloudFilter` / `AlignFilter`
11. `IDecoder` / `IDecoderFactory`
12. `IFrameSyncer` / `IIMUSyncer`
13. `IParameterProvider`(ROS 解耦)
14. `INotification` / 错误模型
15. 能力查询(`ICapabilityQuery`)

---

## 1. 通用类型

```cpp
// File: camera_hal/common.h
namespace camera_hal {

enum class StreamType {
    Unknown = 0,
    Depth,
    Color,
    ColorLeft, ColorRight,
    Infrared, InfraredLeft, InfraredRight,
    Gyro, Accel, Motion,
    Pose6DoF,
    Safety,
    LabeledPointCloud,
    Occupancy,
    Lidar,
    LaserScan,
};

enum class PixelFormat {
    Unknown = 0,
    Z16,
    Y8, Y16,
    RGB8, BGR8, RGBA8, BGRA8,
    YUYV, UYVY, NV12, NV21,
    RAW8, RAW10, RAW16,
    MJPG, H264, H265,    // 压缩
    Y8I, Y12I,           // 交错双 IR
};

enum class TimestampDomain {
    Hardware,    // 设备时钟
    System,      // 主机系统时钟
    Global,      // SDK 对齐后(若支持)
};

struct StreamIndex {
    StreamType type = StreamType::Unknown;
    int index = 0;

    bool operator==(const StreamIndex& o) const { return type==o.type && index==o.index; }
    bool operator<(const StreamIndex& o) const {
        return type < o.type || (type==o.type && index<o.index);
    }
};

} // namespace camera_hal
```

---

## 2. `IFrame` 家族

```cpp
// File: camera_hal/i_frame.h
namespace camera_hal {

class IFrame {
public:
    virtual ~IFrame() = default;

    virtual StreamType     getStreamType() const = 0;
    virtual StreamIndex    getStreamIndex() const = 0;
    virtual PixelFormat    getFormat() const = 0;

    virtual uint64_t        getTimestampNs() const = 0;
    virtual TimestampDomain getTimestampDomain() const = 0;
    virtual uint64_t        getFrameNumber() const = 0;

    // 原始数据(视图,生命周期 = IFrame 对象)
    virtual const void* getData() const = 0;
    virtual size_t      getDataSize() const = 0;

    // 元数据
    virtual std::string getMetadataJson() const = 0;
    virtual bool        hasMetadata(const std::string& key) const = 0;
    virtual int64_t     getMetadataInt(const std::string& key, int64_t default_val = 0) const = 0;
};

class IVideoFrame : public virtual IFrame {
public:
    virtual int getWidth() const = 0;
    virtual int getHeight() const = 0;
    virtual int getBytesPerPixel() const = 0;
    virtual int getStride() const = 0;
};

class IMotionFrame : public virtual IFrame {
public:
    struct Data { float x, y, z; };
    virtual Data getMotionData() const = 0;
};

class IPoseFrame : public virtual IFrame {
public:
    struct Pose {
        float translation[3];
        float quaternion[4];          // x, y, z, w
        float linear_velocity[3];
        float angular_velocity[3];
        float linear_acceleration[3];
        int   tracker_confidence;     // 0..3
        int   mapper_confidence;      // 0..3
    };
    virtual Pose getPose() const = 0;
};

class IFrameSet {
public:
    virtual ~IFrameSet() = default;

    virtual size_t size() const = 0;
    virtual std::shared_ptr<IFrame> getFrame(const StreamIndex&) const = 0;
    virtual std::vector<std::shared_ptr<IFrame>> getAllFrames() const = 0;
    virtual uint64_t getTimestampNs() const = 0;
};

using FrameCallback    = std::function<void(std::shared_ptr<IFrame>)>;
using FrameSetCallback = std::function<void(std::shared_ptr<IFrameSet>)>;

} // namespace
```

---

## 3. `OptionInfo`

```cpp
// File: camera_hal/i_option.h
namespace camera_hal {

enum class OptionType { Bool, Int, Float, Enum };

struct OptionInfo {
    std::string name;
    std::string description;
    OptionType  type = OptionType::Float;
    float min = 0, max = 0, step = 0, default_value = 0;
    bool is_readonly = false;
    std::map<std::string, float> enum_values;   // Enum 时用
};

using OptionChangedCallback = std::function<void(const std::string& name, float value)>;

} // namespace
```

---

## 4. `ISensor`

```cpp
// File: camera_hal/i_sensor.h
namespace camera_hal {

enum class SensorCategory {
    Video,       // depth / color / IR / safety depth / etc
    Motion,      // accel / gyro
    Pose,        // T265
    Lidar,
    Unknown,
};

struct StreamProfile {
    StreamIndex stream;
    int width = 0;         // 0 = any
    int height = 0;        // 0 = any
    int fps = 0;           // 0 = any
    PixelFormat format = PixelFormat::Unknown;  // Unknown = any

    bool exactMatch(const StreamProfile& o) const;
    bool partialMatch(const StreamProfile& o) const;   // 支持 0 通配
};

class ISensor {
public:
    virtual ~ISensor() = default;

    virtual std::string      getName() const = 0;
    virtual SensorCategory   getCategory() const = 0;

    // Profile
    virtual std::vector<StreamProfile> getSupportedProfiles() const = 0;
    virtual std::vector<StreamProfile> getActiveProfiles() const = 0;

    // 启停
    virtual bool start(const std::vector<StreamProfile>& profiles, FrameCallback cb) = 0;
    virtual void stop() = 0;
    virtual bool isStreaming() const = 0;

    // 选项
    virtual std::vector<OptionInfo> getSupportedOptions() const = 0;
    virtual float getOption(const std::string& name) const = 0;
    virtual void  setOption(const std::string& name, float value) = 0;
    virtual void  setOptionChangedCallback(OptionChangedCallback) = 0;

    // 自动曝光 ROI(可选)
    virtual bool supportsAutoExposureROI() const = 0;
    virtual void setAutoExposureROI(int x1, int y1, int x2, int y2) {}

    // 通知(设备通知,如过热)
    using NotificationCallback = std::function<void(const std::string& desc, int severity)>;
    virtual void setNotificationCallback(NotificationCallback) = 0;
};

// Safety 特化
enum class SafetyMode { Run, Standby, Service };

struct SafetyPresetBlob {
    std::vector<uint8_t> data;     // 二进制透传,HAL 不解析
    int index = 0;
};

class ISafetySensor : public ISensor {
public:
    virtual SafetyMode getSafetyMode() const = 0;
    virtual void setSafetyMode(SafetyMode) = 0;
    virtual SafetyPresetBlob readPreset(int index) = 0;
    virtual void writePreset(int index, const SafetyPresetBlob&) = 0;
};

} // namespace
```

---

## 5. `IDevice`

```cpp
// File: camera_hal/i_device.h
namespace camera_hal {

struct DeviceInfo {
    std::string name;                  // "Gemini 335", "Intel RealSense D455"
    std::string serial_number;
    std::string firmware_version;
    std::string hardware_version;
    std::string sdk_version;
    std::string usb_port;
    std::string ip_address;            // 仅网络设备
    std::string connection_type;       // "USB3.0" / "USB2.0" / "Ethernet" / "GMSL"
    uint16_t    vendor_id = 0;
    uint16_t    product_id = 0;
};

struct IPConfig {
    std::string ip;
    std::string netmask;
    std::string gateway;
    bool dhcp = false;
};

class IDevice {
public:
    virtual ~IDevice() = default;

    virtual DeviceInfo getInfo() const = 0;

    // 硬件控制
    virtual void hardwareReset() = 0;
    virtual void syncHostTime() = 0;

    // 传感器
    virtual std::vector<std::shared_ptr<ISensor>> getSensors() const = 0;
    virtual std::shared_ptr<ISensor> getSensor(const std::string& name) const = 0;
    virtual std::shared_ptr<ISensor> findFirstSensor(SensorCategory) const = 0;

    // Preset / 高级 JSON
    virtual std::vector<std::string> listAvailablePresets() const = 0;
    virtual void loadPreset(const std::string& name) = 0;
    virtual void loadJsonConfig(const std::string& json) = 0;  // RealSense 风格
    virtual std::string exportJsonConfig() const = 0;

    // Firmware(可选)
    virtual bool supportsFirmwareUpdate() const = 0;
    using ProgressCallback = std::function<void(float pct, const std::string& status)>;
    virtual void updateFirmware(const std::string& path, ProgressCallback) = 0;

    // 网络配置(可选)
    virtual bool supportsNetworkInterface() const = 0;
    virtual IPConfig getNetworkConfig() const = 0;
    virtual void setNetworkConfig(const IPConfig&, bool force_ip = false) = 0;

    // Interleave(可选)
    virtual bool supportsInterleave() const = 0;
    virtual void setInterleaveEnabled(bool) = 0;

    // 能力集成
    virtual std::shared_ptr<class ICalibration>  getCalibration() = 0;
    virtual std::shared_ptr<class ISyncManager>  getSyncManager() = 0;

    // 通知
    using NotificationCallback = std::function<void(const std::string& desc, int severity)>;
    virtual void setNotificationCallback(NotificationCallback) = 0;
};

} // namespace
```

---

## 6. `ICalibration`

```cpp
// File: camera_hal/i_calibration.h
namespace camera_hal {

enum class DistortionModel {
    None,
    BrownConrady,          // plumb_bob / opencv
    InverseBrownConrady,
    KannalaBrandt4,        // fisheye
};

struct Intrinsics {
    int width = 0, height = 0;
    float fx = 0, fy = 0, ppx = 0, ppy = 0;
    DistortionModel distortion_model = DistortionModel::None;
    float distortion_coeffs[5] = {0};
};

struct Extrinsics {
    // 列主序(Eigen 风格)
    float rotation[9] = { 1,0,0, 0,1,0, 0,0,1 };
    float translation[3] = {0};
};

struct IMUCalibration {
    StreamType stream = StreamType::Unknown;
    float scale_bias[12] = {0};     // 3x4 矩阵
    float noise_variances[3] = {0};
    float bias_variances[3]  = {0};
    bool valid = false;
};

class ICalibration {
public:
    virtual ~ICalibration() = default;

    // 内参
    virtual Intrinsics getIntrinsics(const StreamProfile&) = 0;

    // 外参
    virtual Extrinsics getExtrinsics(const StreamProfile& from,
                                      const StreamProfile& to) = 0;

    // 深度元数据
    virtual float getDepthScale() = 0;     // 米/单位
    virtual float getDepthMinMeters() = 0;
    virtual float getDepthMaxMeters() = 0;

    // IMU
    virtual IMUCalibration getIMUCalibration(StreamType) = 0;

    // 用户标定
    virtual bool supportsUserCalibration() = 0;
    virtual void loadUserCalibration(const std::string& yaml_path) = 0;
    virtual std::string exportUserCalibration() = 0;

    // 在线校准(可选)
    virtual bool supportsOnChipCalibration() = 0;
    virtual void runOnChipCalibration(const std::string& json_config,
                                      IDevice::ProgressCallback cb) = 0;
};

} // namespace
```

---

## 7. `ISyncManager`

```cpp
// File: camera_hal/i_sync_manager.h
namespace camera_hal {

enum class SyncMode {
    FreeRun,
    Standalone,
    Primary,
    Secondary,
    SecondarySynced,
    SoftwareTrigger,
    HardwareTrigger,
};

struct SyncConfig {
    SyncMode mode = SyncMode::FreeRun;
    int depth_delay_us = 0;
    int color_delay_us = 0;
    int trigger2image_delay_us = 0;
    int trigger_out_delay_us = 0;
    bool trigger_out_enabled = false;
    int frames_per_trigger = 1;
};

class ISyncManager {
public:
    virtual ~ISyncManager() = default;

    virtual std::vector<SyncMode> getSupportedSyncModes() const = 0;
    virtual void setSyncConfig(const SyncConfig&) = 0;
    virtual SyncConfig getSyncConfig() const = 0;

    virtual bool supportsSoftwareTrigger() const = 0;
    virtual void triggerOnce() = 0;

    virtual bool isSynced() const = 0;
};

} // namespace
```

---

## 8. `IDeviceManager`

```cpp
// File: camera_hal/i_device_manager.h
namespace camera_hal {

struct DeviceFilter {
    std::string serial_number;
    std::string usb_port;
    std::string ip_address;
    int         net_port = 0;
    std::string device_type_regex;
    int         device_index = -1;    // -1 = 自动
    std::chrono::milliseconds wait_timeout{-1};
};

class IDeviceManager {
public:
    virtual ~IDeviceManager() = default;

    virtual std::vector<DeviceInfo> queryDevices() = 0;
    virtual std::shared_ptr<IDevice> openDevice(const DeviceFilter&) = 0;

    using DeviceChangedCallback = std::function<void(
        const std::vector<DeviceInfo>& added,
        const std::vector<DeviceInfo>& removed)>;
    virtual void setDeviceChangedCallback(DeviceChangedCallback) = 0;

    // 网络设备
    virtual void enableNetworkDeviceEnumeration(bool) = 0;
    virtual std::shared_ptr<IDevice> connectNetworkDevice(
        const std::string& ip, int port) = 0;

    // 进程锁(后端可选实现)
    virtual void enableProcessLock(const std::string& lock_name = "camera_hal") {}

    // 工厂
    static std::shared_ptr<IDeviceManager> create(const std::string& backend);
    // backend: "orbbec" / "realsense" / "rosbag:<path>" / "auto"
};

} // namespace
```

---

## 9. `IFilter` 与 `IFilterPipeline`

```cpp
// File: camera_hal/i_filter.h
namespace camera_hal {

class IFilter {
public:
    virtual ~IFilter() = default;

    virtual std::string getName() const = 0;
    virtual bool isEnabled() const = 0;
    virtual void setEnabled(bool) = 0;

    virtual std::shared_ptr<IFrame>    process(std::shared_ptr<IFrame>) = 0;
    virtual std::shared_ptr<IFrameSet> process(std::shared_ptr<IFrameSet>) = 0;

    virtual std::vector<OptionInfo> getSupportedOptions() const = 0;
    virtual float getOption(const std::string& name) const = 0;
    virtual void  setOption(const std::string& name, float value) = 0;

    virtual std::vector<StreamType> getApplicableStreams() const = 0;
};

class IFilterPipeline {
public:
    virtual ~IFilterPipeline() = default;

    virtual void addFilter(std::shared_ptr<IFilter>) = 0;
    virtual void addFilter(std::shared_ptr<IFilter>, size_t position) = 0;
    virtual void removeFilter(const std::string& name) = 0;
    virtual std::vector<std::shared_ptr<IFilter>> getFilters() const = 0;
    virtual std::shared_ptr<IFilter> getFilter(const std::string& name) const = 0;

    virtual std::shared_ptr<IFrameSet> process(std::shared_ptr<IFrameSet>) = 0;

    static std::shared_ptr<IFilterPipeline> createDefault(std::shared_ptr<IDevice>);
};

// 特化
class PointCloudFilter : public IFilter {
public:
    virtual void setColorSource(const StreamIndex& color_stream) = 0;
    virtual void setUseOrganized(bool) = 0;
    virtual void setMaxDistance(float meters) = 0;
};

class AlignFilter : public IFilter {
public:
    virtual void setAlignTarget(const StreamIndex& target) = 0;
    virtual bool supportsHardwareAlign() const = 0;
    virtual void setHardwareAlign(bool) = 0;
};

class FlipMirrorRotateFilter : public IFilter {
public:
    virtual void setFlipVertical(bool) = 0;
    virtual void setMirrorHorizontal(bool) = 0;
    virtual void setRotation(int degrees) = 0;  // 0/90/180/270
    virtual void setTargetStream(const StreamIndex&) = 0;
};

class UndistortFilter : public IFilter {
public:
    virtual void setTargetStream(const StreamIndex&) = 0;
    virtual void setKeepOriginalSize(bool) = 0;
};

} // namespace
```

---

## 10. `IDecoder` 与工厂

```cpp
// File: camera_hal/i_decoder.h
namespace camera_hal {

class IDecoder {
public:
    virtual ~IDecoder() = default;
    virtual std::string getName() const = 0;
    virtual bool isHardwareAccelerated() const = 0;
    virtual std::vector<PixelFormat> getSupportedInputFormats() const = 0;
    virtual PixelFormat getOutputFormat() const = 0;

    // 同步解码,输出 buffer 由调用方提供
    virtual bool decode(const void* input, size_t input_size,
                         int width, int height,
                         void* output, size_t output_capacity,
                         int& output_stride) = 0;
};

class IDecoderFactory {
public:
    virtual ~IDecoderFactory() = default;

    virtual std::vector<std::string> listAvailableDecoders(PixelFormat input_format) = 0;
    virtual std::shared_ptr<IDecoder> createDecoder(
        PixelFormat input_format,
        const std::string& preferred_decoder = "") = 0;

    using DecoderCreator = std::function<std::shared_ptr<IDecoder>()>;
    virtual void registerDecoder(const std::string& name,
                                   DecoderCreator creator,
                                   std::vector<std::string> platforms,
                                   std::vector<PixelFormat> formats) = 0;

    static IDecoderFactory& instance();
};

} // namespace
```

---

## 11. `IFrameSyncer` / `IIMUSyncer`

```cpp
// File: camera_hal/i_frame_syncer.h
namespace camera_hal {

class IFrameSyncer {
public:
    virtual ~IFrameSyncer() = default;
    virtual void submitFrame(std::shared_ptr<IFrame>) = 0;
    virtual void setCallback(FrameSetCallback) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    static std::shared_ptr<IFrameSyncer> createDefault();
    static std::shared_ptr<IFrameSyncer> createPassthrough();
};

enum class IMUSyncMethod { None, Copy, LinearInterpolation };

class IIMUSyncer {
public:
    virtual ~IIMUSyncer() = default;
    virtual void submitFrame(std::shared_ptr<IMotionFrame>) = 0;

    using SyncedCallback = std::function<void(
        const IMotionFrame::Data& accel,
        const IMotionFrame::Data& gyro,
        uint64_t timestamp_ns)>;
    virtual void setCallback(SyncedCallback) = 0;

    virtual void pause() = 0;
    virtual void resume() = 0;

    static std::shared_ptr<IIMUSyncer> create(IMUSyncMethod);
};

} // namespace
```

---

## 12. `IParameterProvider`(ROS 解耦桥)

```cpp
// File: camera_hal/i_parameter_provider.h
namespace camera_hal {

// HAL 层仅依赖此抽象,不知 ROS 是否存在
class IParameterProvider {
public:
    virtual ~IParameterProvider() = default;

    // 声明 + 初始值 + setter(可选 getter,用于 SDK→ROS 同步)
    virtual void declareFloat(const std::string& name, const OptionInfo& info,
                               float initial,
                               std::function<void(float)> setter,
                               std::function<float()> getter = nullptr) = 0;

    virtual void declareBool(const std::string& name, const std::string& desc,
                              bool initial, std::function<void(bool)> setter) = 0;

    virtual void declareInt(const std::string& name, const OptionInfo& info,
                             int initial, std::function<void(int)> setter) = 0;

    virtual void declareString(const std::string& name, const std::string& desc,
                                const std::string& initial,
                                std::function<void(const std::string&)> setter) = 0;

    virtual void updateValue(const std::string& name, float value) = 0;
    virtual void removeParameter(const std::string& name) = 0;
};

// 默认实现:NoOpParameterProvider(测试用)
// ROS2 实现:在 camera_hal_ros_adapter 里,基于 rclcpp::Node + 异步更新线程

} // namespace
```

---

## 13. `INotification` 与错误模型

```cpp
// File: camera_hal/notification.h
namespace camera_hal {

enum class Severity { Debug, Info, Warn, Error, Fatal };

struct Notification {
    Severity severity = Severity::Info;
    std::string category;      // "device" / "sensor" / "sync" / ...
    std::string description;
    uint64_t timestamp_ns = 0;
};

using NotificationCallback = std::function<void(const Notification&)>;

// HAL 异常层次
class HalException : public std::exception {
public:
    explicit HalException(std::string msg, int code = -1);
    const char* what() const noexcept override;
    int code() const noexcept;
};

class NotSupportedException  : public HalException { public: using HalException::HalException; };
class DeviceNotFoundException : public HalException { public: using HalException::HalException; };
class DeviceBusyException    : public HalException { public: using HalException::HalException; };
class InvalidArgumentException : public HalException { public: using HalException::HalException; };

} // namespace
```

---

## 14. 能力查询

```cpp
// File: camera_hal/i_capability_query.h
namespace camera_hal {

class ICapabilityQuery {
public:
    virtual ~ICapabilityQuery() = default;

    // 字符串化的能力名(利于扩展)
    virtual bool hasCapability(const std::string& name) const = 0;
    virtual std::vector<std::string> listCapabilities() const = 0;
};

// 约定的能力名常量
namespace caps {
    constexpr const char* FirmwareUpdate       = "firmware.update";
    constexpr const char* OnChipCalibration    = "calib.on_chip";
    constexpr const char* UserCalibration      = "calib.user_yaml";
    constexpr const char* HardwareAlign        = "filter.align.hardware";
    constexpr const char* HardwareSync         = "sync.hardware";
    constexpr const char* SoftwareTrigger      = "sync.software_trigger";
    constexpr const char* NetworkDevice        = "device.network";
    constexpr const char* ForceIP              = "device.force_ip";
    constexpr const char* Interleave           = "device.interleave";
    constexpr const char* SafetySensor         = "sensor.safety";
    constexpr const char* LidarSensor          = "sensor.lidar";
    constexpr const char* PoseStream           = "stream.pose6dof";
    constexpr const char* AdvancedMode         = "config.advanced_json";
    constexpr const char* PresetConfig         = "config.preset";
    constexpr const char* HdrMerge             = "filter.hdr_merge";
    constexpr const char* PointCloudGPU        = "point_cloud.gpu";
}

} // namespace
```

`IDevice` 继承 `ICapabilityQuery`,其他接口也可按需。

---

## 15. 接口调用关系图

```
┌──────────────────────────────────────────────────────────────┐
│            用户业务代码 / ROS Adapter                          │
└─────────────┬────────────────────────────────────────────────┘
              │ create
              ▼
     ┌──────────────────┐  queryDevices/openDevice
     │ IDeviceManager   │──────────────────────────┐
     └──────────────────┘                          │
                                                   ▼
                                          ┌──────────────┐
                                          │   IDevice    │◄────── ICapabilityQuery
                                          └───┬──────────┘
                                              │ getSensors / getCalibration / getSyncManager
                            ┌─────────────────┼──────────────────┐
                            ▼                 ▼                  ▼
                    ┌─────────────┐   ┌────────────────┐  ┌──────────────┐
                    │   ISensor   │   │ ICalibration    │  │ ISyncManager │
                    └──┬──────────┘   └────────────────┘  └──────────────┘
                       │ start/stop(FrameCallback)
                       ▼
                ┌──────────────┐
                │   IFrame    │
                │ (Video/IMU/  │
                │  Pose)       │
                └────┬─────────┘
                     │
                     ▼
            ┌─────────────────┐ add/process    ┌────────────┐
            │ IFilterPipeline │───────────────▶│  IFilter   │
            └─────────────────┘                └────────────┘
                     │
                     ▼
            ┌─────────────────┐
            │ IFrameSyncer    │ (optional)
            └─────────────────┘

            ┌─────────────────┐
            │   IDecoder      │ (used internally by ISensor.start)
            └─────────────────┘

            ┌──────────────────────┐
            │ IParameterProvider   │ (ROS Adapter wraps HAL)
            └──────────────────────┘
```

---

## 16. 设计不变量(Invariants)

HAL 任何后端实现都必须满足:

1. **`IFrame` 的生命周期独立于回调**:用户可 shared_ptr 保存到异步 publisher,过 100ms 依然可读 `getData`。
2. **`FrameCallback` 在后端线程执行**:用户代码必须短小非阻塞;沉重处理自己投递到工作线程。
3. **`ISensor::start` 幂等**:重复调用 → 先 stop 再 start;参数完全一致时可空操作。
4. **`IDevice::hardwareReset` 使所有 shared_ptr<ISensor>/ICalibration 失效**:文档明确约束。
5. **`StreamProfile` 是值类型**,复制/传递无副作用;不引用 SDK 内部对象。
6. **`IFilter::process` 不得原地修改入参**:返回新 `IFrame`/`IFrameSet`;SDK 内部可复用 buffer(引用计数),但用户不可假设。
7. **`IOption::setOption` 可能异步生效**:实现可 queue;`getOption` 立即读返回当前 SDK 值(可能还是旧值)。
8. **所有 timestamp 为纳秒 uint64**:避免浮点精度损失。
9. **非抛异常接口清单**:`getInfo`, `getTimestampNs`, `isEnabled`, `supportsXxx` 等查询函数不抛异常;`setOption`, `start`, `openDevice` 等可抛 `HalException` 子类。
10. **后端插件 ABI**:core 版本 `MAJOR` 变化 → 所有插件重编。

---

## 17. 使用示例

```cpp
// 最小使用流程
auto dm = camera_hal::IDeviceManager::create("auto");
DeviceFilter filter;
filter.serial_number = "AY3K41100C3";
auto dev = dm->openDevice(filter);

auto calib = dev->getCalibration();
auto depth_sensor = dev->findFirstSensor(SensorCategory::Video);

// 配置 profile
StreamProfile p{{StreamType::Depth, 0}, 848, 480, 30, PixelFormat::Z16};
depth_sensor->start({p}, [](std::shared_ptr<IFrame> frame){
    auto vf = std::dynamic_pointer_cast<IVideoFrame>(frame);
    // ... use
});

// 动态改参
depth_sensor->setOption("laser_power", 150.0f);

// 应用滤波
auto pipe = IFilterPipeline::createDefault(dev);
pipe->getFilter("spatial")->setEnabled(true);

// 停机
depth_sensor->stop();
```

这份接口定义是本仓库所有分析的**收敛点**——实现它,你就得到了一个可以同时承载 OrbbecSDK_ROS2 v2.7.6 与 realsense-ros 4.57.7 全部核心能力的硬件相机抽象层。
