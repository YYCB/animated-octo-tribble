# ROS2 Service 接口完整分析

## 1. 自定义服务类型定义

### orbbec_camera_msgs/srv/

| 服务类型 | 请求字段 | 响应字段 |
|---------|---------|---------|
| GetBool | — | `bool data` |
| GetInt32 | — | `int32 data` |
| GetString | — | `string data` |
| SetInt32 | `int32 data` | — |
| SetString | `string data` | — |
| GetDeviceInfo | — | `DeviceInfo device_info` |
| GetCameraInfo | — | `sensor_msgs/CameraInfo info` |
| SetFilter | `string filter_name; bool enable; int32 value` | `bool success` |
| SetArrays | `int32[] data` | `bool success` |
| GetUserCalibParams | — | 用户标定参数 |
| SetUserCalibParams | 用户标定参数 | — |

---

## 2. 完整 Service 列表

### 2.1 设备信息

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/get_device_info` | GetDeviceInfo | 设备名称、序列号、固件版本 |
| `/{ns}/get_sdk_version` | GetString | OrbbecSDK 版本号 |

### 2.2 曝光控制

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/get_color_exposure` | GetInt32 | 获取彩色曝光值 |
| `/{ns}/set_color_exposure` | SetInt32 | 设置彩色曝光值 |
| `/{ns}/get_depth_exposure` | GetInt32 | 获取深度曝光值 |
| `/{ns}/set_depth_exposure` | SetInt32 | 设置深度曝光值 |
| `/{ns}/get_ir_exposure` | GetInt32 | 获取 IR 曝光值 |
| `/{ns}/set_ir_exposure` | SetInt32 | 设置 IR 曝光值 |
| `/{ns}/set_color_auto_exposure` | SetBool | 彩色自动曝光开关 |
| `/{ns}/set_depth_auto_exposure` | SetBool | 深度自动曝光开关 |
| `/{ns}/set_ir_auto_exposure` | SetBool | IR 自动曝光开关 |
| `/{ns}/set_color_ae_roi` | SetArrays | 彩色 AE 感兴趣区域 [l,t,r,b] |
| `/{ns}/set_depth_ae_roi` | SetArrays | 深度 AE 感兴趣区域 |

### 2.3 增益控制

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/get_color_gain` | GetInt32 | 获取彩色增益 |
| `/{ns}/set_color_gain` | SetInt32 | 设置彩色增益 |
| `/{ns}/get_depth_gain` | GetInt32 | 获取深度增益 |
| `/{ns}/set_depth_gain` | SetInt32 | 设置深度增益 |
| `/{ns}/get_ir_gain` | GetInt32 | 获取 IR 增益 |
| `/{ns}/set_ir_gain` | SetInt32 | 设置 IR 增益 |

### 2.4 白平衡控制

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/get_white_balance` | GetInt32 | 获取白平衡值 |
| `/{ns}/set_white_balance` | SetInt32 | 设置白平衡值 |
| `/{ns}/get_auto_white_balance` | GetInt32 | 自动白平衡状态 |
| `/{ns}/set_auto_white_balance` | SetBool | 自动白平衡开关 |

### 2.5 激光 / 结构光

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/set_laser_enable` | SetBool | 激光发射器开关 |
| `/{ns}/get_laser_status` | GetBool | 获取激光状态 |
| `/{ns}/set_ldp_enable` | SetBool | LDP 安全保护开关 |
| `/{ns}/get_ldp_status` | GetBool | 获取 LDP 状态 |

### 2.6 图像处理

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/set_color_mirror` | SetBool | 彩色镜像 |
| `/{ns}/set_depth_mirror` | SetBool | 深度镜像 |
| `/{ns}/set_ir_mirror` | SetBool | IR 镜像 |
| `/{ns}/set_color_flip` | SetBool | 彩色翻转 |
| `/{ns}/set_depth_flip` | SetBool | 深度翻转 |
| `/{ns}/set_color_rotation` | SetInt32 | 彩色旋转（0/90/180/270°） |
| `/{ns}/set_filter` | SetFilter | 设置深度滤波器 |

### 2.7 流控制

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/toggle_color` | SetBool | 开关彩色流 |
| `/{ns}/toggle_depth` | SetBool | 开关深度流 |
| `/{ns}/toggle_ir` | SetBool | 开关 IR 流 |
| `/{ns}/set_streams_enable` | SetBool | 全部流开关 |
| `/{ns}/get_streams_enable` | GetBool | 获取流状态 |
| `/{ns}/switch_ir_camera` | SetString | 切换 IR 相机（左/右） |

### 2.8 存储

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/save_images` | Empty | 保存当前帧图像到文件 |
| `/{ns}/save_point_cloud` | Empty | 保存点云到 PLY 文件 |

### 2.9 设备管理

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/reboot_device` | Empty | 重启设备 |
| `/{ns}/set_floor_enable` | SetBool | 地板检测开关 |
| `/{ns}/set_fan_work_mode` | SetInt32 | 风扇工作模式 |

### 2.10 同步与时间

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/set_ptp_config` | SetBool | PTP 时间同步开关 |
| `/{ns}/get_ptp_config` | GetBool | PTP 配置状态 |
| `/{ns}/set_reset_timestamp` | SetBool | 重置设备时间戳 |
| `/{ns}/set_sync_host_time` | SetBool | 主机时间同步开关 |
| `/{ns}/send_software_trigger` | SetBool | 发送软件触发信号 |
| `/{ns}/set_interleave_laser_sync` | SetInt32 | 交错激光同步 |

### 2.11 点云控制

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/set_point_cloud_decimation` | SetInt32 | 点云降采样因子 |
| `/{ns}/get_point_cloud_decimation` | GetInt32 | 获取降采样因子 |

### 2.12 标定参数

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/write_customer_data` | SetString | 写入自定义标定数据 |
| `/{ns}/read_customer_data` | GetString | 读取自定义标定数据 |
| `/{ns}/get_user_calib_params` | GetUserCalibParams | 获取用户标定参数 |
| `/{ns}/set_user_calib_params` | SetUserCalibParams | 设置用户标定参数 |

### 2.13 其他

| Service | 类型 | 说明 |
|---------|------|------|
| `/{ns}/set_ae_mode` | SetString | 设置 AE 模式 |
| `/{ns}/set_sports_mode` | SetBool | 运动模式开关 |
| `/{ns}/set_ir_long_exposure` | SetBool | IR 长曝光开关 |
| `/{ns}/get_lrm_measure_distance` | GetInt32 | LRM 测距值 |

---

## 3. Service 实现模式

### 3.1 流式工厂模式

```cpp
// 按 stream_index_pair 为每个流创建独立 Service
for (auto& stream : IMAGE_STREAMS) {
    if (enable_stream_[stream]) {
        set_exposure_srv_[stream] = node_->create_service<SetInt32>(
            "set_" + stream_name + "_exposure",
            [this, stream](auto req, auto res) {
                setExposureCallback(req, res, stream);
            });
    }
}
```

### 3.2 SDK 属性操作包装

```cpp
void setExposureCallback(request, response, stream_index) {
    auto sensor = sensors_[stream_index];
    sensor->setIntProperty(OB_PROP_EXPOSURE_INT, request->data);
}
```
