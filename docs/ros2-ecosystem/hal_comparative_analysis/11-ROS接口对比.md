# 11 — ROS 接口对比 (msg / srv / action / topic)

> **本章回答**:两家暴露了哪些 ROS 接口?Topic 命名约定?自定义消息有哪些?Service 用来做什么?Action 用来做什么?HAL ROS Adapter 应该提供怎样的接口面?

---

## 1. Orbbec 侧:30+ topic, 40+ service, 0 action

### 1.1 自定义消息(orbbec_camera_msgs)

```
DeviceInfo.msg       设备名/序列号/固件版本/SDK 版本/硬件版本
DeviceStatus.msg     帧率/延迟/在线状态/连接类型/标定状态
Extrinsics.msg       外参 (rotation 9, translation 3) 行主序
IMUInfo.msg          IMU 校准信息(暴露给 SLAM)
Metadata.msg         JSON 帧元数据
RGBD.msg             — 无,Orbbec 没有 RGBD 组合消息
```

⚠️ Orbbec **没有** `RGBD.msg`,要做 RGBD 需用应用层 `message_filters`。

### 1.2 自定义服务(orbbec_camera_srv) — 通用模板

```
GetBool.srv      ---  bool data
GetInt32.srv     ---  int32 data
GetString.srv    ---  string data
SetInt32.srv     int32 data ---
SetString.srv    string data ---
SetBool.srv      bool data ---
SetArrays.srv    int32[] data --- bool success
SetFilter.srv    string filter_name; bool enable; int32 value --- bool success
GetDeviceInfo.srv     --- DeviceInfo device_info
GetCameraInfo.srv     --- sensor_msgs/CameraInfo info
GetUserCalibParams    --- 用户标定参数
SetUserCalibParams    用户标定参数 ---
```

通过模板复用,大部分硬件 option 都用 GetInt32 / SetInt32 暴露。

### 1.3 完整 Service 列表(节选,完整 40+)

```
设备:
  /camera/get_device_info
  /camera/get_sdk_version

曝光:
  /camera/{get,set}_color_exposure
  /camera/{get,set}_depth_exposure
  /camera/{get,set}_ir_exposure
  /camera/set_{color,depth,ir}_auto_exposure
  /camera/set_{color,depth}_ae_roi

增益:
  /camera/{get,set}_color_gain
  /camera/{get,set}_depth_gain
  /camera/{get,set}_ir_gain

白平衡:
  /camera/{get,set}_white_balance
  /camera/{get,set}_auto_white_balance

激光:
  /camera/set_laser_enable
  /camera/get_laser_status
  /camera/set_ldp_enable
  /camera/get_ldp_status
  /camera/set_laser_power_level

镜像/翻转:
  /camera/set_color_mirror
  /camera/set_depth_mirror
  /camera/set_color_flip
  /camera/set_depth_flip

设备控制:
  /camera/reboot_device
  /camera/save_image
  /camera/save_point_cloud
  /camera/toggle_color
  /camera/toggle_depth
  /camera/toggle_ir
  /camera/trigger_capture          (软件触发)

滤波器:
  /camera/set_filter
  /camera/set_spatial_filter
  /camera/set_temporal_filter
  /camera/set_hole_filling_filter
  /camera/set_decimation_filter
```

### 1.4 Topic 列表(节选)

```
图像:
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/color/metadata
  /camera/depth/image_raw
  /camera/depth/camera_info
  /camera/depth/metadata
  /camera/ir/image_raw
  /camera/ir/camera_info
  /camera/ir_left/image_raw          # 双目设备
  /camera/ir_right/image_raw

点云:
  /camera/depth/points                  # XYZ
  /camera/depth_registered/points       # XYZRGB

IMU:
  /camera/accel/sample
  /camera/gyro/sample
  /camera/gyro_accel/sample             # 硬件同步组合

外参:
  /camera/depth_to_color/extrinsics    (latched)
  /camera/depth_to_ir/extrinsics       (latched)

设备状态:
  /camera/device_status                 # DeviceStatus 消息,周期发布
  /camera/filter_status                 # JSON 字符串,所有滤波器配置

LiDAR:
  /camera/scan                          # LaserScan
  /camera/points                        # PointCloud2
```

### 1.5 Action

❌ 无。所有操作都是同步 service,即使长耗时(固件更新)也是。

---

## 2. RealSense 侧:多 topic, 10 service, 1 action

### 2.1 自定义消息(realsense2_camera_msgs)

```
Extrinsics.msg       float64[9] rotation (列主序), float64[3] translation
IMUInfo.msg          header, float64[12] data (3x4), noise/bias 方差
Metadata.msg         header, string json_data
RGBD.msg             header, color_info, depth_info, color, depth (对齐后)
```

### 2.2 自定义服务

```
DeviceInfo.srv             --- name, serial, firmware, usb_type, sensors, port
CalibConfigRead.srv        --- string calib_config (JSON), bool success
CalibConfigWrite.srv       string calib_config --- bool success
SafetyPresetRead.srv       int32 index --- SafetyPreset preset, bool success
SafetyPresetWrite.srv      int32 index, SafetyPreset preset --- bool success
SafetyInterfaceConfigRead.srv --- ... bool success
SafetyInterfaceConfigWrite.srv ... --- bool success
SerializedDeviceConfigRead.srv  --- string config
SerializedDeviceConfigWrite.srv string config --- bool success
TriggeredCalibration.srv   ... --- bool success
```

数量上少于 Orbbec(10 vs 40+),因为大部分硬件 option 走 ROS 参数(见 [第 5 章](05-参数系统对比.md))。

### 2.3 Action

```
TriggeredCalibration.action
  goal:    string json_config
  result:  bool success, float health
  feedback: float progress, string status
```

用于在线校准(Tare / On-chip),长耗时 ~10s。

### 2.4 Topic 列表(节选)

```
图像:
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/color/metadata
  /camera/depth/image_raw
  /camera/depth/camera_info
  /camera/depth/metadata
  /camera/infra1/image_raw
  /camera/infra2/image_raw
  /camera/aligned_depth_to_color/image_raw    # 深度对齐到彩色
  /camera/aligned_depth_to_color/camera_info

点云:
  /camera/depth/color/points                  # XYZRGB

IMU:
  /camera/accel/sample
  /camera/gyro/sample
  /camera/imu                                  # 软件融合
  /camera/accel/imu_info                       # latched
  /camera/gyro/imu_info                        # latched
  /camera/motion/sample                        # D555 motion 流

外参:
  /camera/extrinsics/depth_to_color           (latched)
  /camera/extrinsics/depth_to_infra1          (latched)

RGBD 组合:
  /camera/rgbd                                # 自定义 RGBD.msg

安全相机(D585S):
  /camera/labeled_pointcloud                  # 带标签点云
  /camera/occupancy                           # 占据栅格
  /camera/safety_log
```

### 2.5 Image Publisher 双实现

```
image_rcl_publisher        → rclcpp::Publisher<sensor_msgs::msg::Image>
                             (支持 intra-process 零拷贝)

image_transport_publisher  → image_transport::Publisher
                             (支持 compressed/theora 等插件)
```

可通过参数或编译宏切换。Orbbec 也有 `image_publisher` 抽象,但没有 RealSense 这么显式的两路实现。

---

## 3. 对比总览

| 维度 | Orbbec | RealSense |
|---|---|---|
| 自定义 msg | 5(无 RGBD) | 4(含 RGBD) |
| 自定义 srv | **40+** | 10 |
| Action | 0 | 1 |
| 硬件 option 接口 | Service | **ROS Param**(自动映射) |
| 设备状态 | DeviceStatus topic | 通过参数 + diagnostic_updater |
| 滤波器状态 | 独立 JSON topic | 通过参数 |
| 外参格式 | Extrinsics(行主序) | Extrinsics(列主序) |
| RGBD 组合 | ❌ | ✅ RGBD.msg |
| Aligned depth topic | ❌(数据通过 D2C 嵌入 depth) | ✅ 独立 topic |
| Image 发布器 | `image_publisher` 一种 | **双实现可切** |
| Diagnostic | 通过 DeviceStatus | 标准 `diagnostic_msgs::DiagnosticArray` |
| Safety 相机 | ❌ | ✅ 6 个 service + 2 topic |
| LiDAR 接口 | ✅ scan/points | ❌ |

### ⚠️ 关键差异

1. **硬件 option 接口风格**:
   - Orbbec 走 service —— 用户调 `set_color_exposure`。
   - RealSense 走 param —— 用户调 `ros2 param set rgb_camera.exposure 100`。
   - **HAL 决策**:统一 RealSense 风格(更标准、可发现、可脚本化)。Orbbec 后端通过自动映射达到同样效果。

2. **DeviceStatus 单独 topic vs Diagnostic 标准**:
   - Orbbec 用自定义 DeviceStatus 消息,字段固定。
   - RealSense 用 ROS 标准 `diagnostic_msgs`(可被 rqt_runtime_monitor 之类工具直接观察)。
   - **HAL 决策**:Adapter 提供两种,默认用 ROS 标准 diagnostic。

3. **Action 利用**:
   - 仅 RealSense 用 Action。任何长耗时(固件更新、在线校准)都应该 Action。
   - **HAL 决策**:`IDevice::updateFirmware` / `ICalibration::runOnChipCalibration` 接受 `ProgressCallback`,Adapter 包成 Action。

---

## 4. 对 HAL ROS Adapter 的接口面建议

### 4.1 推荐 Topic 列表

```
图像 (per stream):
  ~/{stream}/image_raw                    sensor_msgs/Image
  ~/{stream}/camera_info                  sensor_msgs/CameraInfo
  ~/{stream}/metadata                     custom Metadata
  ~/aligned_depth_to_{stream}/image_raw   (启用对齐时)
  ~/aligned_depth_to_{stream}/camera_info

点云:
  ~/depth/points                          sensor_msgs/PointCloud2 (XYZ)
  ~/depth/color/points                    sensor_msgs/PointCloud2 (XYZRGB)

IMU:
  ~/accel/sample                          sensor_msgs/Imu
  ~/gyro/sample                           sensor_msgs/Imu
  ~/imu                                   sensor_msgs/Imu (融合或硬件同步)
  ~/accel/imu_info                        custom IMUInfo (latched)
  ~/gyro/imu_info                         custom IMUInfo (latched)

外参:
  ~/extrinsics/{from}_to_{to}             custom Extrinsics (latched)

RGBD 组合(可选):
  ~/rgbd                                  custom RGBD

LiDAR(可选):
  ~/scan                                  sensor_msgs/LaserScan
  ~/points                                sensor_msgs/PointCloud2

诊断:
  /diagnostics                            diagnostic_msgs/DiagnosticArray (周期)
```

### 4.2 推荐 Service 列表(精简)

```
~/get_device_info                  --- 自定义 DeviceInfo
~/reset_device                     --- 触发 hardwareReset()
~/save_image                       string filename ---
~/save_point_cloud                 string filename ---
~/trigger_capture                  --- 软件触发(SyncMode=SoftwareTrigger 时)
~/load_calibration                 string yaml_path ---
~/get_calibration                  --- string json
```

⚠️ 不要给每个硬件 option 写 service。让用户用 `ros2 param`。

### 4.3 推荐 Action 列表

```
~/update_firmware                  string fw_path  →  progress / success
~/run_calibration                  string config   →  progress / health / success
```

### 4.4 自定义消息(精简)

```
camera_hal_msgs/msg/Extrinsics      (行主序,与 Orbbec 兼容)
camera_hal_msgs/msg/IMUInfo
camera_hal_msgs/msg/Metadata
camera_hal_msgs/msg/RGBD
camera_hal_msgs/msg/DeviceInfo
camera_hal_msgs/msg/DeviceStatus    (备用,主要走 diagnostic_msgs)
```

### 4.5 Topic 命名规范

```
~/...                              所有相对节点 namespace
{stream}                           "color" / "depth" / "infra1" / "infra2" / "ir_left" / ...
{from}_to_{to}                     "depth_to_color" / "depth_to_infra1" / ...
```

避免使用 Orbbec 的 `_registered` 后缀,改用 `aligned_depth_to_color` 这种 RealSense 风格更清晰。

### 4.6 QoS 默认

| Topic | QoS |
|---|---|
| `*/image_raw` | `SENSOR_DATA` (best_effort, depth=5) |
| `*/camera_info` | `DEFAULT` (reliable, depth=10) |
| `*/extrinsics` | `latched` (transient_local, keep_last=1) |
| `*/imu_info` | `latched` |
| `*/sample` (IMU) | `SENSOR_DATA` |
| `*/points` | `SENSOR_DATA` |
| `/diagnostics` | `DEFAULT` |

每流 QoS 可通过参数覆盖,参考 RealSense 的 `{stream}.image_qos` 模式。

### 4.7 Adapter 类结构

```cpp
class CameraHalRosAdapter {
public:
    CameraHalRosAdapter(RosNodeBase& node, std::shared_ptr<IDevice>);

    void start();   // 创建发布器/订阅器/服务/Action,startSensors
    void stop();

private:
    // HAL → ROS
    void onVideoFrame(std::shared_ptr<IVideoFrame>);
    void onMotionFrame(std::shared_ptr<IMotionFrame>);
    void onSyncedImu(...);
    void publishStaticTransforms();
    void publishDiagnostics();

    // ROS → HAL
    void onParameterChange(const Parameter&);
    void onTriggerService(...);
    void onUpdateFirmwareAction(...);

    // 资源
    std::map<StreamIndex, ImagePublisher> _image_pubs;
    std::map<StreamIndex, rclcpp::Publisher<CameraInfo>> _info_pubs;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf;
    std::shared_ptr<diagnostic_updater::Updater> _diag;

    // HAL
    std::shared_ptr<IDevice> _device;
    std::vector<std::shared_ptr<ISensor>> _sensors;
    std::shared_ptr<IFilterPipeline> _filter_pipeline;
    std::shared_ptr<IFrameSyncer> _frame_syncer;       // optional
    std::shared_ptr<IIMUSyncer> _imu_syncer;           // optional
    std::shared_ptr<ICalibration> _calib;
};
```

### 4.8 坑

- ⚠️ **Topic 数量爆炸**:每流 4-5 个 topic × 6 流 = 30 个。HAL Adapter 应根据实际启用的 stream 创建/销毁,而不是预先全开。
- ⚠️ **latched topic 与 transient_local**:ROS2 中应使用 `transient_local + keep_last(1)` 而非 ROS1 的 latched。
- ⚠️ **QoS 不匹配**:发布者 best_effort 而订阅者 reliable 会无法通信。文档要明确各 topic 的 QoS。
- ⚠️ **CameraInfo 与 image_raw 时间戳必须一致**:同一帧 publish 时同时发,且 `header.stamp` 完全相同(ROS image_pipeline 强制要求)。
- ⚠️ **Lifecycle 模式下 publisher 状态**:`LifecycleNode` 的 publisher 在 inactive 状态会被禁用,Adapter 的 start/stop 必须遵循生命周期。
