# 数据流与 Topic 分析

## 1. Topic 完整列表

### 1.1 图像 Topic

| Topic 名称 | 消息类型 | 说明 |
|-----------|---------|------|
| `/{camera_name}/color/image_raw` | sensor_msgs/Image | 彩色图像原始数据 |
| `/{camera_name}/color/camera_info` | sensor_msgs/CameraInfo | 彩色相机内参 |
| `/{camera_name}/depth/image_raw` | sensor_msgs/Image | 深度图像（16UC1，mm） |
| `/{camera_name}/depth/camera_info` | sensor_msgs/CameraInfo | 深度相机内参 |
| `/{camera_name}/ir/image_raw` | sensor_msgs/Image | IR 红外图像 |
| `/{camera_name}/ir/camera_info` | sensor_msgs/CameraInfo | IR 相机内参 |
| `/{camera_name}/ir_left/image_raw` | sensor_msgs/Image | 左 IR（双目设备） |
| `/{camera_name}/ir_right/image_raw` | sensor_msgs/Image | 右 IR（双目设备） |
| `/{camera_name}/color_left/image_raw` | sensor_msgs/Image | 左彩色（Gemini 305 等） |
| `/{camera_name}/color_right/image_raw` | sensor_msgs/Image | 右彩色（Gemini 305 等） |

### 1.2 点云 Topic

| Topic 名称 | 消息类型 | 说明 |
|-----------|---------|------|
| `/{camera_name}/depth/points` | sensor_msgs/PointCloud2 | 深度点云（XYZ） |
| `/{camera_name}/depth_registered/points` | sensor_msgs/PointCloud2 | 彩色点云（XYZRGB） |

### 1.3 IMU Topic

| Topic 名称 | 消息类型 | 说明 |
|-----------|---------|------|
| `/{camera_name}/gyro/sample` | sensor_msgs/Imu | 陀螺仪数据 |
| `/{camera_name}/accel/sample` | sensor_msgs/Imu | 加速度计数据 |
| `/{camera_name}/gyro_accel/sample` | sensor_msgs/Imu | 同步融合 IMU 数据 |

### 1.4 外参与元数据 Topic

| Topic 名称 | 消息类型 | 说明 |
|-----------|---------|------|
| `/{camera_name}/depth_to_color/extrinsics` | Extrinsics | 深度到彩色外参 |
| `/{camera_name}/depth_to_ir/extrinsics` | Extrinsics | 深度到 IR 外参 |
| `/{camera_name}/color/metadata` | Metadata | 彩色帧元数据（JSON） |
| `/{camera_name}/depth/metadata` | Metadata | 深度帧元数据（JSON） |

### 1.5 设备状态 Topic

| Topic 名称 | 消息类型 | 说明 |
|-----------|---------|------|
| `/{camera_name}/device_status` | DeviceStatus | 帧率、延迟、在线状态 |
| `/{camera_name}/filter_status` | std_msgs/String | 滤波器状态（JSON） |

### 1.6 LiDAR 专用 Topic

| Topic 名称 | 消息类型 | 说明 |
|-----------|---------|------|
| `/{camera_name}/scan` | sensor_msgs/LaserScan | 2D 激光扫描 |
| `/{camera_name}/points` | sensor_msgs/PointCloud2 | LiDAR 3D 点云 |

---

## 2. 数据流详细分析

### 2.1 图像数据流

```
硬件传感器
    │
    ▼
OrbbecSDK Pipeline（SDK 内部）
    │（回调）
    ▼
onNewFrameSetCallback(FrameSet)
    │
    ├── 提取 Color Frame
    │   ├── MJPEG  → JPEGDecoder（CPU）/ JetsonNVDecoder / RkMppDecoder
    │   ├── YUYV / UYVY / NV21 / NV12 → FormatConvertFilter → BGR/RGB
    │   ├── H264 / H265 → 硬件解码器
    │   └── RGB / BGR   → 直接使用
    │   ▼
    │   processColorFrameFilter() → 颜色滤波器链
    │   ▼
    │   onNewFrameCallback(COLOR) → 发布
    │
    ├── 提取 Depth Frame
    │   ▼
    │   processDepthFrameFilter() → 深度滤波器链（12 种）
    │   ▼
    │   onNewFrameCallback(DEPTH) → 发布
    │
    ├── 提取 IR Frame(s) → onNewFrameCallback(INFRA0/INFRA1/INFRA2)
    │
    └── publishPointCloud(FrameSet)
        ├── publishDepthPointCloud()    # XYZ
        └── publishColoredPointCloud()  # XYZRGB（需要 depth_registration）
```

### 2.2 时间戳处理

```cpp
// 三种时间域模式
uint64_t getFrameTimestampUs(frame) {
    if (time_domain_ == "device")  return frame->timeStampUs();
    if (time_domain_ == "system")  return frame->systemTimeStampUs();
    /* global */                   return frame->globalTimeStampUs();
}
```

### 2.3 QoS 配置映射

```
"SYSTEM_DEFAULT"  → rmw_qos_profile_system_default
"DEFAULT"         → rmw_qos_profile_default
"PARAMETER_EVENTS"→ rmw_qos_profile_parameter_events
"SERVICES_DEFAULT"→ rmw_qos_profile_services_default
"PARAMETERS"      → rmw_qos_profile_parameters
"SENSOR_DATA"     → rmw_qos_profile_sensor_data
```

### 2.4 图像编码映射

| SDK 格式（OBFormat） | ROS 编码（encoding） | 步长倍数 |
|---------------------|---------------------|---------|
| OB_FORMAT_Y16  | mono16（16UC1） | 2 |
| OB_FORMAT_Y8   | mono8（8UC1）   | 1 |
| OB_FORMAT_YUYV | yuv422_yuy2     | 2 |
| OB_FORMAT_RGB  | rgb8            | 3 |
| OB_FORMAT_BGR  | bgr8            | 3 |
| OB_FORMAT_MJPG | bgr8（解码后）  | 3 |
| OB_FORMAT_BGRA | bgra8           | 4 |

---

## 3. 点云生成流程

### 3.1 深度点云（Depth-only）

```
Depth Frame
    │
    ▼
获取校准参数（OBCalibrationParam）
    │
    ▼
计算 XY 查找表（OBXYTables，一次性缓存复用）
    │
    ▼
ob::PointCloudFilter.setCreatePointFormat(OB_FORMAT_POINT)
    │
    ▼
filter.process(depthFrame) → ob::PointsFrame
    │
    ▼
构建 PointCloud2 消息（XYZ float32）→ depth_cloud_pub_->publish()
```

### 3.2 彩色点云（XYZRGB）

```
FrameSet（Depth + Color）
    │
    ▼
对齐（Align Filter）
    ├── HW：硬件对齐（ALIGN_D2C_HW_MODE）
    └── SW：软件对齐（align_filter_->process()）
    │
    ▼
ob::PointCloudFilter.setCreatePointFormat(OB_FORMAT_RGB_POINT)
    │
    ▼
filter.process(frameSet) → PointCloud2（XYZRGB）→ depth_registration_cloud_pub_->publish()
```

---

## 4. image_transport 集成

通过 `image_transport` 支持多种传输插件：
- **raw**：原始数据传输（默认）
- **compressed**：JPEG / PNG 压缩
- **theora**：Theora 视频编码

自定义 `image_publisher` 类额外支持 intra-process 零拷贝发布。

---

## 5. TF 坐标系

### 5.1 坐标系树

```
{camera_name}_link（base_link）
    ├── {camera_name}_depth_frame
    │   └── {camera_name}_depth_optical_frame
    ├── {camera_name}_color_frame
    │   └── {camera_name}_color_optical_frame
    ├── {camera_name}_ir_frame
    │   └── {camera_name}_ir_optical_frame
    ├── {camera_name}_ir_left_frame（双目）
    │   └── {camera_name}_ir_left_optical_frame
    ├── {camera_name}_ir_right_frame（双目）
    │   └── {camera_name}_ir_right_optical_frame
    ├── {camera_name}_gyro_frame
    │   └── {camera_name}_gyro_optical_frame
    └── {camera_name}_accel_frame
        └── {camera_name}_accel_optical_frame
```

### 5.2 外参转换
- SDK 提供 `OBExtrinsic`（3×3 旋转 + 3×1 平移）
- 旋转矩阵 → 四元数：`rotationMatrixToQuaternion()`
- 静态 TF：`StaticTransformBroadcaster`
- 动态 TF：`TransformBroadcaster`（按指定频率发布）
