# 08 — IMU 数据处理对比

> **本章回答**:IMU(陀螺仪 + 加速度计) 怎么从设备出来?accel + gyro 怎么对齐?有没有"硬件同步"和"软件融合"两种模式?HAL 的 IMU 接口该怎么设计?

---

## 1. Orbbec 侧:独立 IMU Pipeline,可硬件同步输出

### 1.1 IMU 数据模型

```cpp
struct IMUData {
    stream_index_pair stream_;    // GYRO 或 ACCEL
    Eigen::Vector3d   data_;      // (x, y, z)
    double            timestamp_; // 纳秒
    bool isSet() const { return timestamp_ >= 0; }
};
```

### 1.2 IMU 流类型

| 流 | SDK 类型 | 单位 |
|---|---|---|
| ACCEL | `OB_STREAM_ACCEL` | m/s² |
| GYRO | `OB_STREAM_GYRO` | rad/s |

### 1.3 两种启动模式

**A. 独立模式 (`startIMU`)**
```
startIMU()
  ├── 获取 Accel sensor → 配置采样率/量程 → 启动
  │     └─ callback: onNewIMUFrameCallback(frame, ACCEL)
  └── 获取 Gyro sensor → 配置采样率/量程 → 启动
        └─ callback: onNewIMUFrameCallback(frame, GYRO)
```

**B. 同步模式 (`startIMUSyncStream`)** — Gemini 330 等设备支持
```
startIMUSyncStream()
  ├── enable_sync_output_accel_gyro_ = true
  ├── 配置 IMU Pipeline (单独 ob::Pipeline)
  └── callback: onNewIMUFrameSyncOutputCallback(accel_frame, gyro_frame)
       └─ 设备硬件级保证 accel/gyro 时间对齐
```

### 1.4 数据流(独立模式)

```
onNewIMUFrameCallback(frame, stream_index)
   │
   ├── 提取时间戳 (getFrameTimestampUs)
   ├── 提取 3 轴数据
   │    ├── AccelFrame: accelValue.x/y/z (m/s²)
   │    └── GyroFrame:  gyroValue.x/y/z  (rad/s)
   │
   ├── 转 sensor_msgs::msg::Imu 消息
   │
   └── 发布到对应 topic
        ├── /camera/accel/sample
        └── /camera/gyro/sample
```

### 1.5 数据流(同步模式)

```
onNewIMUFrameSyncOutputCallback(accel_frame, gyro_frame)
   │
   ├── 时间戳已硬件对齐
   ├── 构建组合 sensor_msgs::msg::Imu (linear_acceleration + angular_velocity)
   └── 发布到 /camera/gyro_accel/sample
```

### 1.6 同步参数

```cpp
parameters_->setParam("enable_sync_output_accel_gyro", false);  // 是否启用硬件同步
parameters_->setParam("accel_rate", "OB_SAMPLE_RATE_500_HZ");
parameters_->setParam("accel_range", "OB_ACCEL_FS_2g");
parameters_->setParam("gyro_rate", "OB_SAMPLE_RATE_500_HZ");
parameters_->setParam("gyro_range", "OB_GYRO_FS_500dps");
```

### 1.7 IMU 与图像 Pipeline 的隔离

IMU 走 **独立** `ob::Pipeline`,与 Color/Depth/IR 主 Pipeline 解耦。好处:
- IMU 高频(500Hz~1kHz)不与图像低频(30fps)在同一 SDK 队列竞争。
- IMU 失效(传感器没响应)不影响图像流。

代价:**主 Pipeline 与 IMU Pipeline 之间没有硬件级时间对齐**(只有共享设备时钟)。

---

## 2. RealSense 侧:统一回调 + 软件融合

### 2.1 IMU 流

| 流 | SDK 类型 | 单位 |
|---|---|---|
| ACCEL | `RS2_STREAM_ACCEL` | m/s² |
| GYRO | `RS2_STREAM_GYRO` | rad/s |
| MOTION (D555 等) | `RS2_STREAM_MOTION` | 已对齐组合 |

### 2.2 回调路径

```
rs2::sensor (motion module) 帧回调
   │
   ▼
imu_callback_function = [this](rs2::frame f) {
    imu_callback(f);                    // 独立发布(总是)
    if (_imu_sync_method != NONE)
        imu_callback_sync(f);            // 软件融合(条件)
};
```

### 2.3 独立发布 (`imu_callback`)

```
imu_callback(frame)
  ├── 提取 accel 或 gyro 数据
  ├── 构建 sensor_msgs::msg::Imu
  │    ├── 仅填充 angular_velocity (gyro) 或 linear_acceleration (accel)
  │    ├── 协方差矩阵(参数 angular_velocity_cov / linear_accel_cov)
  └── 发布
       ├── /camera/accel/sample
       └── /camera/gyro/sample
```

### 2.4 软件融合 (`imu_callback_sync`)

通过 `unite_imu_method` 参数选择:

| 值 | 名称 | 算法 |
|---|---|---|
| 0 | NONE | 不融合 |
| 1 | COPY | 简单复制最近样本 |
| 2 | LINEAR_INTERPOLATION | 线性插值 |

```cpp
// SyncedImuPublisher 内部逻辑
class SyncedImuPublisher {
    std::deque<rs2::motion_frame> _accel_history;
    std::deque<rs2::motion_frame> _gyro_history;

    void onNewFrame(rs2::motion_frame f) {
        if (f.is_accel()) _accel_history.push(f);
        else              _gyro_history.push(f);

        // 配对
        while (canPair()) {
            auto accel = popAccel();
            auto gyro  = popGyro();
            // 时间戳对齐 (LINEAR_INTERPOLATION)
            //   插值出 accel 在 gyro 时间戳的值,反之亦然
            sensor_msgs::msg::Imu imu;
            imu.angular_velocity = gyro.data;
            imu.linear_acceleration = interpolated_accel;
            publish(imu);
        }
    }

    void Pause();   // 由 frame_callback 在处理 frameset 时调用,避免与图像时间错乱
    void Resume();
};
```

### 2.5 IMU 协方差

通过 ROS 参数提供,默认 `0.01`:
```
linear_accel_cov: 0.01
angular_velocity_cov: 0.01
```

填充到 `sensor_msgs::msg::Imu::linear_acceleration_covariance` 等字段,作为 EKF 等下游组件的输入。

### 2.6 IMU Calibration 信息发布

```cpp
// 一次性发布到 /camera/accel/imu_info 和 /camera/gyro/imu_info
// 使用 realsense2_camera_msgs::msg::IMUInfo
//   data: float64[12]            # 3x4 校准矩阵
//   noise_variances: float64[3]
//   bias_variances:  float64[3]
// QoS: latched (transient_local)
```

下游可订阅一次性获取 IMU 内参。

### 2.7 hold_back_imu_for_frames

参数:让 IMU 发布 **暂停** 直到首个图像帧到来,确保下游(如 VIO)不会拿到没有图像匹配的孤立 IMU 数据。

---

## 3. 对比总览

| 维度 | Orbbec | RealSense |
|---|---|---|
| Pipeline | **独立** IMU pipeline | 与视频 sensor 共用 SDK,独立回调 |
| 硬件同步 | **支持**(`startIMUSyncStream`) | ❌(只有软件融合) |
| 软件融合 | ❌(只在硬件同步下输出 united) | **支持**(COPY / LINEAR_INTERPOLATION) |
| 独立发布 | `/accel/sample` `/gyro/sample` | `/accel/sample` `/gyro/sample` |
| 组合发布 | `/gyro_accel/sample` (硬件同步) | `/imu` (软件融合) |
| 配置参数 | `accel_rate` / `accel_range` / `gyro_rate` / `gyro_range` | `accel_fps` / `gyro_fps`(range 通过 SDK option 自动映射) |
| 协方差 | 不显式 | `linear_accel_cov` / `angular_velocity_cov` |
| Calibration 信息 | 通过 service 查询 | **`/imu_info` topic latched** |
| 暂停同步 IMU | ❌ | `synced_imu_publisher.Pause/Resume` (frameset 处理时) |
| Motion 流(D555) | ❌ | ✅ (`RS2_STREAM_MOTION`) |

### ⚠️ 关键差异

1. **同步方式分布**:
   - Orbbec 偏向 **硬件同步**(Gemini 330+ 设备固件支持)。
   - RealSense 偏向 **软件融合**(D435i 等没有固件级同步,只能 ROS 端插值)。
   - **HAL 决策**:同时支持两种,通过能力查询接口告诉用户当前设备能否硬件同步。

2. **Calibration 信息暴露方式**:
   - RealSense 用 latched topic(订阅时一定能拿到)。
   - Orbbec 用 service。
   - **HAL 决策**:HAL 接口提供 `getIMUCalibration()` 方法,ROS Adapter 层既可发 latched topic 又可提供 service,二选一或都做。

---

## 4. 对 HAL 的可复用要点

### 4.1 `IMotionFrame` (复用 [第 4 章](04-数据流与帧回调机制对比.md))

```cpp
class IMotionFrame : public IFrame {
public:
    struct Data { float x, y, z; };
    virtual Data getMotionData() const = 0;
};
```

### 4.2 `IDevice::startIMU` 与同步配置

```cpp
struct IMUConfig {
    bool enable_accel = false;
    int  accel_fps = 0;          // 0 = default
    int  accel_range = 0;        // 0 = default; 单位是 g (2/4/8/16)

    bool enable_gyro = false;
    int  gyro_fps = 0;
    int  gyro_range = 0;         // 单位是 dps (250/500/1000/2000)

    bool enable_hardware_sync = false;  // 后端不支持时报错或回退
};

class IDevice {
    // ...
    virtual bool supportsHardwareIMUSync() const = 0;
    virtual void startIMU(const IMUConfig& config, IMUFrameCallback cb) = 0;
    virtual void stopIMU() = 0;
};

using IMUFrameCallback = std::function<void(std::shared_ptr<IMotionFrame>)>;
```

### 4.3 `IIMUSyncer` (软件融合,可选组件)

```cpp
enum class IMUSyncMethod {
    None,
    Copy,                    // 用最近的 accel 样本
    LinearInterpolation,     // 线性插值
};

class IIMUSyncer {
public:
    virtual void submitFrame(std::shared_ptr<IMotionFrame>) = 0;

    using SyncedCallback = std::function<void(
        const IMotionFrame::Data& accel,
        const IMotionFrame::Data& gyro,
        uint64_t timestamp_ns)>;
    virtual void setCallback(SyncedCallback) = 0;

    virtual void pause() = 0;       // 处理图像 frameset 时
    virtual void resume() = 0;

    static std::shared_ptr<IIMUSyncer> create(IMUSyncMethod method);
};
```

### 4.4 `getIMUCalibration` 接口

```cpp
struct IMUCalibration {
    StreamType stream;           // Accel 或 Gyro
    float scale_bias[12];        // 3x4 校准矩阵
    float noise_variances[3];
    float bias_variances[3];
    bool valid;
};

class IDevice {
    // ...
    virtual IMUCalibration getIMUCalibration(StreamType) const = 0;
};
```

### 4.5 ROS Adapter 层的发布策略

```
1. 总是发 /accel/sample 和 /gyro/sample (独立)
2. 如果配置 imu_sync_method != None:
   - 创建 IIMUSyncer
   - 把 frame callback 接到 syncer
   - syncer 输出 → /imu (组合)
3. 如果设备支持 hardware sync 且用户启用:
   - IDevice::startIMU(config{enable_hardware_sync=true})
   - 设备直接给出已对齐 frame 对(通过 IFrameSet 包含 accel + gyro 两个 IMotionFrame)
   - Adapter 直接发 /imu,无 syncer 需求
4. 协方差从参数 linear_accel_cov / angular_velocity_cov 填入
5. /accel/imu_info 和 /gyro/imu_info 用 latched QoS 发布 IMU calibration
6. hold_back_imu_for_frames=true 时,IMU 发布在首个图像帧前 Pause
```

### 4.6 时间戳一致性

HAL 必须保证 IMU `IMotionFrame::getTimestampNs()` 与 Video `IVideoFrame::getTimestampNs()` 在 **同一时钟域**(`getTimestampDomain()` 应一致)。两家 SDK 都遵循,HAL 不能引入时间域错乱。

### 4.7 坑

- ⚠️ **采样率与实际频率**:用户设 500Hz,实际可能 480Hz(SDK round to supported)。HAL 应通过 `getActiveProfiles` 给出真实值。
- ⚠️ **重力分量**:加速度计返回的 z 分量包含重力 ~9.81 m/s²,下游 EKF 需要扣除。HAL 不做这件事(Orbbec/RealSense 都不做),但要在文档中提示。
- ⚠️ **Pause/Resume 的死锁**:`synced_imu_publisher` 的 Pause 调用必须 reentrant,否则 frame_callback 内调 Pause 会等待自己持有的锁。RealSense 通过原子标志位规避,HAL `IIMUSyncer::pause` 应同样无锁。
- ⚠️ **首样本无效**:LinearInterpolation 需要至少 2 个 accel + 2 个 gyro 样本才能输出第一个组合,否则缓存等待。HAL 应允许配置最大缓存深度,防止 OOM。
