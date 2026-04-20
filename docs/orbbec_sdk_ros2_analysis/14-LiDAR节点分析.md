# LiDAR 节点（OBLidarNode）深度分析

## 1. 设备类型判断

```cpp
// OBCameraNodeDriver::initializeDevice() 中：
auto device_type = device_info_->deviceType();
if (device_type == OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA
    || device_type == OB_LIDAR) {
    ob_lidar_node_ = std::make_unique<OBLidarNode>(this, device, parameters_);
    ob_lidar_node_->startStreams();
    ob_lidar_node_->startIMU();
}
```

---

## 2. 支持的 LiDAR 设备

| 设备 | 类型 | 特点 |
|------|------|------|
| Pulsar ME450 | 固态 LiDAR | 多回波、高密度点云 |
| Pulsar SL450 | 固态 LiDAR | 室外使用 |

---

## 3. 发布数据

### 3.1 LaserScan 发布

```cpp
void publishScan(frame_set) {
    auto lidar_frame  = frame_set->getFrame(OB_FRAME_LIDAR);
    auto sphere_points = lidar_frame->data<OBLiDARSpherePoint>();

    sensor_msgs::msg::LaserScan scan;
    scan.angle_min      = deg2rad(min_angle_);    // -135°
    scan.angle_max      = deg2rad(max_angle_);    // +135°
    scan.range_min      = min_range_;             // 0.05 m
    scan.range_max      = max_range_;             // 30.0 m

    for (auto& p : sphere_points) {
        int idx = (p.theta - min_angle_) / angle_increment;
        scan.ranges[idx]      = p.r;
        scan.intensities[idx] = p.intensity;
    }

    filterScan(scan);
    scan_pub_->publish(scan);
}
```

### 3.2 3D 点云发布

```cpp
void publishPointCloud(frame_set) {
    // 球面点 → 笛卡尔点转换
    auto cart_points = spherePointToPoint(sphere_points, count);

    sensor_msgs::msg::PointCloud2 cloud;
    // 填充 x, y, z, intensity 字段
    point_cloud_pub_->publish(cloud);
}
```

### 3.3 球面点云发布

```cpp
void publishSpherePointCloud(frame_set) {
    // 直接使用球面坐标（r, theta, phi, intensity）
    // 填充 PointCloud2 消息
}
```

---

## 4. 球面到笛卡尔坐标转换

```cpp
std::vector<OBLiDARPoint> spherePointToPoint(
    OBLiDARSpherePoint* sphere_point, uint32_t count) {

    std::vector<OBLiDARPoint> points;
    for (uint32_t i = 0; i < count; i++) {
        float r     = sphere_point[i].r;
        float theta = sphere_point[i].theta;  // 水平角
        float phi   = sphere_point[i].phi;    // 垂直角

        OBLiDARPoint p;
        p.x         = r * std::cos(phi) * std::cos(theta);
        p.y         = r * std::cos(phi) * std::sin(theta);
        p.z         = r * std::sin(phi);
        p.intensity = sphere_point[i].intensity;
        points.push_back(p);
    }
    return points;
}
```

---

## 5. 多帧累积

```cpp
// enable_cloud_accumulated_ = true
if (enable_cloud_accumulated_) {
    frame_buffer_.push_back(frame_set);
    if (frame_buffer_.size() >= cloud_accumulation_count_) {
        publishMergedPointCloud();
        frame_buffer_.clear();
    }
}
```

---

## 6. 扫描过滤

```cpp
void filterScan(LaserScan& scan) {
    for (auto& r : scan.ranges) {
        if (r < min_range_ || r > max_range_)
            r = std::numeric_limits<float>::infinity();
    }
}
```

---

## 7. LiDAR 特有参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| lidar_format | string | "ANY" | 数据格式 |
| lidar_rate | int | 0 | 扫描频率 |
| echo_mode | string | "" | 回波模式 |
| min_angle | float | -135.0 | 最小扫描角度（°） |
| max_angle | float | 135.0 | 最大扫描角度（°） |
| min_range | float | 0.05 | 最小测距（m） |
| max_range | float | 30.0 | 最大测距（m） |
| enable_scan_to_point | bool | false | 扫描转点云 |
| enable_cloud_accumulated | bool | false | 点云累积 |
| cloud_accumulation_count | int | -1 | 累积帧数 |
| publish_n_pkts | int | 1 | 多包发布数 |
| vertical_fov | float | -1 | 垂直视场角 |
| filter_level | int | -1 | 过滤级别 |
| repetitive_scan_mode | int | -1 | 重复扫描模式 |

---

## 8. 与 OBCameraNode 对比

| 特性 | OBCameraNode | OBLidarNode |
|------|-------------|-------------|
| 设备类型 | 深度相机 | LiDAR |
| 主要输出 | Image + PointCloud2 | LaserScan + PointCloud2 |
| 数据源 | Color / Depth / IR 传感器 | LiDAR 传感器 |
| 滤波器 | 12 种深度后处理滤波器 | 角度 / 距离过滤 |
| 对齐 | Depth 到 Color 对齐 | 不需要 |
| IMU | 支持 | 支持 |

---

## 9. 对 HAL 的启示

LiDAR 节点表明 HAL 需要支持不同类型的传感器输出：
- 2D 图像流（Color / Depth / IR）
- 3D 点云流（笛卡尔 / 球面）
- 2D 激光扫描（LaserScan）
- IMU 数据流

建议在 `IDevice` 接口中通过 `SensorType` 区分，在 `IFrameSet` 中支持 `LIDAR` 帧类型，并在 HAL 层提供球面 ↔ 笛卡尔的坐标转换工具函数。
