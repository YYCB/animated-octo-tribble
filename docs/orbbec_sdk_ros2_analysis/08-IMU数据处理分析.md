# IMU 数据处理分析

## 1. IMU 数据模型

### 1.1 IMUData 结构

```cpp
struct IMUData {
    stream_index_pair stream_;    // GYRO 或 ACCEL
    Eigen::Vector3d   data_;      // (x, y, z) 3 轴数据
    double            timestamp_; // 纳秒时间戳
    bool isSet() const { return timestamp_ >= 0; }
};
```

### 1.2 IMU 流类型

| 流 | SDK 类型 | 数据内容 |
|----|---------|---------|
| ACCEL | OB_STREAM_ACCEL | 3 轴加速度（m/s²） |
| GYRO  | OB_STREAM_GYRO  | 3 轴角速度（rad/s） |

---

## 2. IMU 启动模式

### 2.1 独立模式（startIMU）

```
startIMU()
    ├── 获取 Accel 传感器 → 配置采样率/量程 → 启动
    │   └── 回调：onNewIMUFrameCallback(frame, ACCEL)
    │
    └── 获取 Gyro 传感器 → 配置采样率/量程 → 启动
        └── 回调：onNewIMUFrameCallback(frame, GYRO)
```

### 2.2 同步模式（startIMUSyncStream）

```
startIMUSyncStream()
    ├── enable_sync_output_accel_gyro_ = true
    ├── 配置 IMU Pipeline
    └── 回调：onNewIMUFrameSyncOutputCallback(accelFrame, gyroFrame)
```

---

## 3. IMU 数据处理流程

### 3.1 独立模式数据流

```
onNewIMUFrameCallback(frame, stream_index)
    │
    ├── 提取时间戳（getFrameTimestampUs）
    ├── 提取 3 轴数据
    │   ├── AccelFrame: accelValue.x/y/z  (m/s²)
    │   └── GyroFrame:  gyroValue.x/y/z   (rad/s)
    │
    ├── 构建 IMUData {stream, data, timestamp}
    │
    ├── 数据校正（可选）
    │   ├── enable_accel_data_correction_
    │   └── enable_gyro_data_correction_
    │
    ├── 线性插值融合（FillImuDataLinearInterpolation）
    │   或直接复制（FillImuDataCopy）
    │
    └── 发布 sensor_msgs::msg::Imu
        ├── header.stamp            = 时间戳
        ├── angular_velocity        = gyro_data
        ├── linear_acceleration     = accel_data
        └── orientation_covariance[0] = -1（无方向估计）
```

### 3.2 同步模式数据流

```
onNewIMUFrameSyncOutputCallback(accelFrame, gyroFrame)
    │
    ├── 提取 Accel + Gyro 数据（同一时间戳）
    ├── 构建 sensor_msgs::msg::Imu
    └── imu_gyro_accel_publisher_->publish(imu_msg)
```

---

## 4. IMU 数据融合算法

### 4.1 线性插值（FillImuDataLinearInterpolation）

目的：将异步的 Accel 和 Gyro 数据在时间轴上对齐。

```
算法步骤：
1. 维护 IMU 历史队列（imu_history_）
2. 每收到新的 Accel / Gyro 数据点
3. 在历史中找到最近两个对偶数据点
4. 线性插值计算对齐时刻的值
5. 合并为统一的 IMU 消息
```

### 4.2 直接复制（FillImuDataCopy）

将最新 Accel 数据与最新 Gyro 数据直接合并，不做时间对齐。适合采样率相同且硬件已同步的场景。

---

## 5. IMU 配置参数

### 5.1 采样率选项

```
"50hz", "100hz", "200hz", "500hz", "1khz", "2khz", "4khz"
```

### 5.2 加速度计量程

```
"2g", "4g", "8g", "16g"
```

### 5.3 陀螺仪量程

```
"16dps", "31dps", "62dps", "125dps", "250dps",
"500dps", "1000dps", "2000dps"
```

---

## 6. IMUInfo 消息

```cpp
orbbec_camera_msgs::msg::IMUInfo createIMUInfo(stream_index) {
    IMUInfo info;
    info.noise_density         = intrinsic.noiseDensity;
    info.random_walk           = intrinsic.randomWalk;
    info.reference_temperature = intrinsic.referenceTemp;
    info.bias                  = {b[0], b[1], b[2]};
    info.gravity               = {g[0], g[1], g[2]};
    info.scale_misalignment    = intrinsic.scaleMisalignment; // 3×3
    info.temperature_slope     = intrinsic.temperatureSlope;  // 3×3
    return info;
}
```

---

## 7. 协方差设置

```cpp
void setDefaultIMUMessage(imu_msg) {
    imu_msg.orientation_covariance[0] = -1;  // 无方向估计

    // 对角协方差矩阵（linear_accel_cov_ 默认 0.0001）
    imu_msg.linear_acceleration_covariance[0] = linear_accel_cov_;
    imu_msg.linear_acceleration_covariance[4] = linear_accel_cov_;
    imu_msg.linear_acceleration_covariance[8] = linear_accel_cov_;

    // 对角协方差矩阵（angular_vel_cov_ 默认 0.0001）
    imu_msg.angular_velocity_covariance[0] = angular_vel_cov_;
    imu_msg.angular_velocity_covariance[4] = angular_vel_cov_;
    imu_msg.angular_velocity_covariance[8] = angular_vel_cov_;
}
```

---

## 8. 对 HAL 的启示

```
IIMU {
    configure(accel_rate, gyro_rate, accel_range, gyro_range) → void
    start(mode: INDEPENDENT | SYNC)      → void
    stop()                               → void
    onData(callback: (IMUData) → void)  → void
    getIntrinsics()                      → IMUIntrinsics
}
```
