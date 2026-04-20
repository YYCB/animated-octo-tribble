# RealSense ROS 4.57.7 源码分析 - HAL 硬件抽象层设计建议

## 1. 设计目标

基于 realsense-ros 4.57.7 和之前调研的 OrbbecSDK_ROS2 的分析，设计一个统一的硬件抽象层 (HAL)，使得：

1. 上层应用（ROS2 节点、自定义应用等）不直接依赖具体的相机 SDK
2. 支持多厂商相机（RealSense、Orbbec、其他深度相机）
3. 支持运行时切换相机类型
4. 保持与原始 SDK 相当的性能

## 2. 整体架构

```
┌──────────────────────────────────────────────────────────┐
│                    Application Layer                      │
│              (ROS2 Node / Custom App / etc.)               │
└──────────────────────────┬───────────────────────────────┘
                           │
┌──────────────────────────┴───────────────────────────────┐
│                    HAL Public API                          │
│  ┌─────────┐  ┌──────────┐  ┌─────────┐  ┌───────────┐  │
│  │ Device   │  │ Sensor   │  │ Stream  │  │ Filter    │  │
│  │ Manager  │  │ Manager  │  │ Pipeline│  │ Pipeline  │  │
│  └─────────┘  └──────────┘  └─────────┘  └───────────┘  │
│  ┌─────────┐  ┌──────────┐  ┌─────────┐  ┌───────────┐  │
│  │ Param   │  │ Calib    │  │ TF/     │  │ Diagnostic│  │
│  │ Provider│  │ Manager  │  │ Extrin. │  │ Provider  │  │
│  └─────────┘  └──────────┘  └─────────┘  └───────────┘  │
└──────────────────────────┬───────────────────────────────┘
                           │
┌──────────────────────────┴───────────────────────────────┐
│                 HAL Backend (Plugin)                       │
│  ┌───────────────┐  ┌──────────────┐  ┌────────────────┐ │
│  │ RealSense     │  │ Orbbec       │  │ Future Camera  │ │
│  │ Backend       │  │ Backend      │  │ Backend        │ │
│  │ (librealsense)│  │ (OrbbecSDK)  │  │               │ │
│  └───────────────┘  └──────────────┘  └────────────────┘ │
└──────────────────────────────────────────────────────────┘
```

## 3. 核心接口定义

### 3.1 设备管理 (IDeviceManager)

```cpp
namespace camera_hal {

struct DeviceInfo {
    std::string serial_number;
    std::string device_name;
    std::string firmware_version;
    std::string physical_port;
    std::string usb_type;
    uint16_t product_id;
    std::string vendor;           // "RealSense", "Orbbec", etc.
};

struct DeviceFilter {
    std::string serial_number;    // 可选
    std::string usb_port;         // 可选
    std::string device_type;      // 可选，正则匹配
};

class IDeviceManager {
public:
    virtual ~IDeviceManager() = default;
    
    // 设备发现
    virtual std::vector<DeviceInfo> queryDevices() = 0;
    virtual std::shared_ptr<IDevice> openDevice(const DeviceFilter& filter) = 0;
    
    // 设备事件
    using DeviceChangedCallback = std::function<void(
        const std::vector<DeviceInfo>& added,
        const std::vector<DeviceInfo>& removed)>;
    virtual void setDeviceChangedCallback(DeviceChangedCallback cb) = 0;
    
    // 工厂方法
    static std::shared_ptr<IDeviceManager> create(const std::string& backend);
    // backend: "realsense", "orbbec", "auto"
};

} // namespace camera_hal
```

### 3.2 设备接口 (IDevice)

```cpp
class IDevice {
public:
    virtual ~IDevice() = default;
    
    virtual DeviceInfo getInfo() = 0;
    virtual std::vector<std::shared_ptr<ISensor>> getSensors() = 0;
    virtual void hardwareReset() = 0;
    
    // 高级配置
    virtual bool supportsAdvancedMode() = 0;
    virtual void loadJsonConfig(const std::string& json) = 0;
    
    // 安全功能（可选）
    virtual bool hasSafetySensor() = 0;
    virtual std::shared_ptr<ISafetySensor> getSafetySensor() = 0;
    
    // 校准功能（可选）
    virtual bool supportsCalibration() = 0;
    virtual std::shared_ptr<ICalibration> getCalibration() = 0;
};
```

### 3.3 传感器接口 (ISensor)

```cpp
enum class SensorType {
    Depth,
    Color,
    Infrared,
    Motion,      // IMU (accel + gyro)
    Safety,
    DepthMapping,
    Unknown
};

struct OptionInfo {
    std::string name;
    std::string description;
    float min, max, step, default_value;
    bool is_readonly;
    enum Type { Bool, Int, Float, Enum } type;
    std::map<std::string, int> enum_values;  // for Enum type
};

class ISensor {
public:
    virtual ~ISensor() = default;
    
    virtual std::string getName() = 0;
    virtual SensorType getType() = 0;
    
    // Profile 管理
    virtual std::vector<StreamProfile> getSupportedProfiles() = 0;
    virtual std::vector<StreamProfile> getActiveProfiles() = 0;
    
    // 启停
    virtual bool start(const std::vector<StreamProfile>& profiles,
                       FrameCallback callback) = 0;
    virtual void stop() = 0;
    
    // 选项
    virtual std::vector<OptionInfo> getSupportedOptions() = 0;
    virtual float getOption(const std::string& name) = 0;
    virtual void setOption(const std::string& name, float value) = 0;
    
    // 通知
    using NotificationCallback = std::function<void(
        const std::string& description, int severity)>;
    virtual void setNotificationCallback(NotificationCallback cb) = 0;
};
```

### 3.4 流配置 (StreamProfile)

```cpp
enum class StreamType {
    Depth, Color, Infrared, Gyro, Accel, Motion,
    Safety, LabeledPointCloud, Occupancy
};

enum class PixelFormat {
    Z16, Y8, Y16, RGB8, BGR8, RGBA8, BGRA8,
    YUYV, UYVY, RAW8, RAW10, RAW16,
    Y8I, Y12I,  // stereo infrared
    None
};

struct StreamProfile {
    StreamType stream_type;
    int stream_index;
    int width;
    int height;
    int fps;
    PixelFormat format;
    
    bool operator==(const StreamProfile& other) const;
};
```

### 3.5 帧数据 (Frame)

```cpp
class Frame {
public:
    virtual ~Frame() = default;
    
    virtual StreamProfile getProfile() = 0;
    virtual double getTimestamp() = 0;
    virtual int getTimestampDomain() = 0;  // 0=hardware, 1=system
    virtual uint64_t getFrameNumber() = 0;
    virtual const void* getData() = 0;
    virtual size_t getDataSize() = 0;
    
    // 元数据
    virtual std::string getMetadataJson() = 0;
};

class VideoFrame : public Frame {
public:
    virtual int getWidth() = 0;
    virtual int getHeight() = 0;
    virtual int getBytesPerPixel() = 0;
    virtual int getStride() = 0;
};

class MotionFrame : public Frame {
public:
    struct MotionData {
        float x, y, z;
    };
    virtual MotionData getData() = 0;
};

class FrameSet {
public:
    virtual std::vector<std::shared_ptr<Frame>> getFrames() = 0;
    virtual std::shared_ptr<Frame> getFrame(StreamType type, int index = 0) = 0;
};

using FrameCallback = std::function<void(std::shared_ptr<Frame>)>;
using FrameSetCallback = std::function<void(std::shared_ptr<FrameSet>)>;
```

### 3.6 滤波器接口 (IFilter)

```cpp
class IFilter {
public:
    virtual ~IFilter() = default;
    
    virtual std::string getName() = 0;
    virtual bool isEnabled() = 0;
    virtual void setEnabled(bool enabled) = 0;
    
    virtual std::shared_ptr<Frame> process(std::shared_ptr<Frame> frame) = 0;
    virtual std::shared_ptr<FrameSet> process(std::shared_ptr<FrameSet> frameset) = 0;
    
    virtual std::vector<OptionInfo> getOptions() = 0;
    virtual float getOption(const std::string& name) = 0;
    virtual void setOption(const std::string& name, float value) = 0;
};

class IFilterPipeline {
public:
    virtual void addFilter(std::shared_ptr<IFilter> filter) = 0;
    virtual void removeFilter(const std::string& name) = 0;
    virtual std::shared_ptr<FrameSet> process(std::shared_ptr<FrameSet> frameset) = 0;
    virtual std::vector<std::shared_ptr<IFilter>> getFilters() = 0;
};
```

### 3.7 校准接口 (ICalibration)

```cpp
struct Intrinsics {
    int width, height;
    float fx, fy;       // 焦距
    float ppx, ppy;     // 主点
    int distortion_model;  // 0=none, 1=brown_conrady, 2=inverse_brown, 3=kannala_brandt4
    float distortion_coeffs[5];
};

struct Extrinsics {
    float rotation[9];     // 3x3 列主序旋转矩阵
    float translation[3];  // 平移（米）
};

struct IMUCalibData {
    float data[12];           // 3x4 校准矩阵
    float noise_variances[3];
    float bias_variances[3];
};

class ICalibration {
public:
    virtual Intrinsics getIntrinsics(const StreamProfile& profile) = 0;
    virtual Extrinsics getExtrinsics(const StreamProfile& from, 
                                      const StreamProfile& to) = 0;
    virtual IMUCalibData getIMUCalibData(StreamType type) = 0;
    
    // 深度特有
    virtual float getDepthScale() = 0;
    
    // 在线校准（可选）
    virtual bool supportsOnChipCalibration() = 0;
    virtual void runOnChipCalibration(const std::string& config,
                                       std::function<void(float progress)> cb) = 0;
};
```

### 3.8 帧同步器 (IFrameSyncer)

```cpp
class IFrameSyncer {
public:
    virtual ~IFrameSyncer() = default;
    
    virtual void submitFrame(std::shared_ptr<Frame> frame) = 0;
    virtual void setCallback(FrameSetCallback callback) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
};
```

## 4. RealSense Backend 实现要点

### 4.1 设备管理器映射

| HAL | realsense-ros | librealsense2 |
|-----|---------------|---------------|
| `IDeviceManager::queryDevices()` | `RSContextSingletonWrapper::query_devices()` | `rs2::context::query_devices()` |
| `IDeviceManager::openDevice()` | `RealSenseNodeFactory::getDevice()` | 遍历 `device_list` |
| `IDevice::getSensors()` | `dev.query_sensors()` | `rs2::device::query_sensors()` |
| `ISensor::start()` | `RosSensor::start()` | `rs2::sensor::open()` + `start()` |
| `ISensor::getOption()` | `sensor.get_option()` | `rs2::options::get_option()` |

### 4.2 数据流映射

| HAL | realsense-ros | librealsense2 |
|-----|---------------|---------------|
| `FrameCallback` | `frame_callback_function` | `rs2::frame_callback` |
| `FrameSet` | `rs2::frameset` | `rs2::frameset` |
| `VideoFrame` | `rs2::video_frame` | `rs2::video_frame` |
| `MotionFrame` | `rs2::motion_frame` | `rs2::motion_frame` |
| `IFrameSyncer` | `rs2::asynchronous_syncer` | `rs2::asynchronous_syncer` |

### 4.3 滤波器映射

| HAL | realsense-ros | librealsense2 |
|-----|---------------|---------------|
| `DecimationFilter` | `NamedFilter(rs2::decimation_filter)` | `rs2::decimation_filter` |
| `SpatialFilter` | `NamedFilter(rs2::spatial_filter)` | `rs2::spatial_filter` |
| `AlignFilter` | `AlignDepthFilter` | `rs2::align` |
| `PointCloudFilter` | `PointcloudFilter` | `rs2::pointcloud` |

## 5. Orbbec Backend 实现要点（参考 OrbbecSDK_ROS2）

| HAL | OrbbecSDK_ROS2 | OrbbecSDK |
|-----|----------------|-----------|
| `IDeviceManager` | `OBCameraNodeFactory` | `ob::Context` |
| `IDevice` | `OBCameraNode` | `ob::Device` |
| `ISensor` | 内部管理 | `ob::Sensor` |
| `FrameCallback` | `frameCallback` | `ob::FrameCallback` |

## 6. ROS2 适配层设计

HAL 本身不依赖 ROS2，需要一个适配层将 HAL 的输出转换为 ROS2 话题：

```cpp
class CameraHALRosAdapter {
public:
    CameraHALRosAdapter(rclcpp::Node& node, std::shared_ptr<IDevice> device);
    
    void start();
    void stop();
    
private:
    // 将 HAL 帧转为 ROS 消息
    void onVideoFrame(std::shared_ptr<VideoFrame> frame);
    void onMotionFrame(std::shared_ptr<MotionFrame> frame);
    
    // 参数同步
    void syncParametersFromHAL();
    void syncParametersToHAL();
    
    // TF 管理
    void publishTransforms();
    
    // ROS 发布器
    std::map<StreamType, image_transport::Publisher> _image_pubs;
    std::map<StreamType, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> _info_pubs;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_broadcaster;
};
```

## 7. 与 realsense-ros 的关键差异点

| 方面 | realsense-ros 现状 | HAL 建议 |
|------|-------------------|----------|
| SDK 耦合 | 直接使用 `rs2::*` 类型 | 通过接口隔离 |
| ROS 耦合 | 深度绑定 ROS2 API | HAL 层无 ROS 依赖 |
| 设备差异 | 单一 `BaseRealSenseNode` | 后端 Plugin 处理 |
| 参数系统 | 绑定 ROS 参数服务器 | `IParameterProvider` 抽象 |
| 多厂商 | 仅 RealSense | 可扩展后端 |
| 滤波器 | 绑定 RS2 滤波器 | 通用 `IFilter` 接口 |

## 8. 实现路径建议

### 阶段 1：核心框架
1. 定义 HAL 接口（本文档中的接口）
2. 实现 RealSense Backend（基于 librealsense2）
3. 实现基本的 ROS2 适配层

### 阶段 2：功能完善
4. 实现 Orbbec Backend（基于 OrbbecSDK）
5. 实现完整的参数系统
6. 实现滤波器管道
7. 实现 TF/外参管理

### 阶段 3：高级功能
8. 实现多设备支持
9. 实现 Lifecycle 支持
10. 实现安全相机功能
11. 实现在线校准
12. 性能优化（零拷贝、GPU 加速）

## 9. 性能考虑

### 9.1 零拷贝
- realsense-ros 在 intra-process 模式下使用 `UniquePtr` 实现零拷贝
- HAL 应该支持将 SDK 帧数据直接映射为输出，避免额外拷贝

### 9.2 内存管理
- Frame 对象使用 `shared_ptr` 管理生命周期
- SDK 帧引用计数需要正确管理

### 9.3 线程安全
- 帧回调来自 SDK 内部线程
- 参数更新来自 ROS 回调线程
- 需要适当的同步机制（mutex、condition_variable）
