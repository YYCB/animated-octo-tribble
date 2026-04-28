# RealSense ROS 4.57.7 源码分析 - Launch 与配置系统

## 1. 概述

Launch 系统负责 ROS2 节点的启动配置，支持命令行参数、YAML 配置文件和多相机编排。

### 文件位置
- `realsense2_camera/launch/rs_launch.py` - 主 Launch 文件
- `realsense2_camera/launch/rs_multi_camera_launch.py` - 多相机 Launch
- `realsense2_camera/launch/rs_multi_camera_launch_sync.py` - 多相机同步 Launch
- `realsense2_camera/launch/rs_intra_process_demo_launch.py` - Intra-process 演示

## 2. 主 Launch 文件 (`rs_launch.py`)

### 2.1 可配置参数完整列表

```python
configurable_parameters = [
    # === 设备配置 ===
    {'name': 'camera_name',          'default': 'camera',   'description': '相机唯一名称'},
    {'name': 'camera_namespace',     'default': 'camera',   'description': '命名空间'},
    {'name': 'serial_no',            'default': "''",       'description': '序列号'},
    {'name': 'usb_port_id',          'default': "''",       'description': 'USB端口'},
    {'name': 'device_type',          'default': "''",       'description': '设备类型'},
    {'name': 'config_file',          'default': "''",       'description': 'YAML配置文件'},
    {'name': 'json_file_path',       'default': "''",       'description': '高级配置JSON'},
    {'name': 'initial_reset',        'default': 'false',    'description': '初始重置'},
    {'name': 'rosbag_filename',      'default': "''",       'description': 'Rosbag文件'},
    {'name': 'rosbag_loop',          'default': 'false',    'description': '循环播放'},
    
    # === GPU 加速 ===
    {'name': 'accelerate_gpu_with_glsl', 'default': 'false', 'description': 'GLSL加速'},
    
    # === 日志与输出 ===
    {'name': 'log_level',            'default': 'info',     'description': '日志级别'},
    {'name': 'output',               'default': 'screen',   'description': '输出方式'},
    
    # === 彩色流 ===
    {'name': 'enable_color',         'default': 'true'},
    {'name': 'rgb_camera.color_profile', 'default': '0,0,0', 'description': 'width,height,fps'},
    {'name': 'rgb_camera.color_format',  'default': 'RGB8'},
    {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true'},
    
    # === 深度流 ===
    {'name': 'enable_depth',         'default': 'true'},
    {'name': 'depth_module.depth_profile', 'default': '0,0,0'},
    {'name': 'depth_module.depth_format',  'default': 'Z16'},
    {'name': 'depth_module.exposure',      'default': '8500'},
    {'name': 'depth_module.gain',          'default': '16'},
    {'name': 'depth_module.enable_auto_exposure', 'default': 'true'},
    
    # === 红外流 ===
    {'name': 'enable_infra',         'default': 'false'},
    {'name': 'enable_infra1',        'default': 'false'},
    {'name': 'enable_infra2',        'default': 'false'},
    {'name': 'depth_module.infra_profile',  'default': '0,0,0'},
    {'name': 'depth_module.infra_format',   'default': 'RGB8'},
    {'name': 'depth_module.infra1_format',  'default': 'Y8'},
    {'name': 'depth_module.infra2_format',  'default': 'Y8'},
    
    # === HDR ===
    {'name': 'depth_module.hdr_enabled',    'default': 'false'},
    {'name': 'depth_module.exposure.1',     'default': '7500'},
    {'name': 'depth_module.gain.1',         'default': '16'},
    {'name': 'depth_module.exposure.2',     'default': '1'},
    {'name': 'depth_module.gain.2',         'default': '16'},
    
    # === IMU ===
    {'name': 'enable_gyro',          'default': 'false'},
    {'name': 'enable_accel',         'default': 'false'},
    {'name': 'enable_motion',        'default': 'false'},
    {'name': 'gyro_fps',             'default': '0'},
    {'name': 'accel_fps',            'default': '0'},
    {'name': 'motion_fps',           'default': '0'},
    {'name': 'unite_imu_method',     'default': '0'},    # 0=None, 1=Copy, 2=Interpolation
    
    # === 同步与组合 ===
    {'name': 'enable_sync',          'default': 'false'},
    {'name': 'enable_rgbd',          'default': 'false'},
    {'name': 'depth_module.inter_cam_sync_mode', 'default': '0'},  # 0=Default, 1=Master, 2=Slave
    
    # === 滤波器 ===
    {'name': 'align_depth.enable',       'default': 'false'},
    {'name': 'colorizer.enable',         'default': 'false'},
    {'name': 'decimation_filter.enable', 'default': 'false'},
    {'name': 'spatial_filter.enable',    'default': 'false'},
    {'name': 'temporal_filter.enable',   'default': 'false'},
    {'name': 'disparity_filter.enable',  'default': 'false'},
    {'name': 'hole_filling_filter.enable','default': 'false'},
    {'name': 'hdr_merge.enable',         'default': 'false'},
    {'name': 'rotation_filter.enable',   'default': 'false'},
    {'name': 'rotation_filter.rotation', 'default': '0.0'},
    {'name': 'decimation_filter.filter_magnitude', 'default': '2'},
    
    # === 点云 ===
    {'name': 'pointcloud.enable',            'default': 'false'},
    {'name': 'pointcloud.stream_filter',     'default': '2'},    # 纹理源
    {'name': 'pointcloud.stream_index_filter','default': '0'},
    {'name': 'pointcloud.ordered_pc',        'default': 'false'},
    {'name': 'pointcloud.allow_no_texture_points', 'default': 'false'},
    
    # === TF ===
    {'name': 'publish_tf',           'default': 'true'},
    {'name': 'tf_publish_rate',      'default': '0.0'},
    {'name': 'base_frame_id',        'default': 'link'},
    {'name': 'tf_prefix',            'default': ''},
    
    # === 其他 ===
    {'name': 'clip_distance',        'default': '-2.'},
    {'name': 'angular_velocity_cov', 'default': '0.01'},
    {'name': 'linear_accel_cov',     'default': '0.01'},
    {'name': 'diagnostics_period',   'default': '0.0'},
    {'name': 'wait_for_device_timeout', 'default': '-1.'},
    {'name': 'reconnect_timeout',    'default': '6.'},
    
    # === 安全相机 ===
    {'name': 'enable_safety',        'default': 'false'},
    {'name': 'safety_camera.safety_mode', 'default': '0'},  # 0=Run, 1=Standby, 2=Service
    
    # === 深度映射 ===
    {'name': 'enable_labeled_point_cloud', 'default': 'false'},
    {'name': 'depth_mapping_camera.labeled_point_cloud_profile', 'default': '0,0,0'},
    {'name': 'enable_occupancy',     'default': 'false'},
    {'name': 'depth_mapping_camera.occupancy_profile', 'default': '0,0,0'},
]
```

### 2.2 Launch 逻辑

```python
def launch_setup(context, params, param_name_suffix=''):
    # 1. 加载配置文件
    config_file = LaunchConfiguration('config_file').perform(context)
    params_from_file = {} if config_file == "''" else yaml_to_dict(config_file)
    
    # 2. 参数验证（警告不支持的参数）
    for param in command_line_params:
        if param not in supported_params:
            print(f"Warning: '{param}' is not supported")
    
    # 3. 检查 Lifecycle 模式
    lifecycle_params = yaml_to_dict('config/global_settings.yaml')
    use_lifecycle = lifecycle_params.get("use_lifecycle_node", False)
    
    # 4. 选择节点类型
    node_action = LifecycleNode if use_lifecycle else Node
    
    # 5. 创建节点
    return [
        node_action(
            package='realsense2_camera',
            namespace=camera_namespace,
            name=camera_name,
            executable='realsense2_camera_node',
            parameters=[params, params_from_file],  # 合并参数
            output=output,
            arguments=['--ros-args', '--log-level', log_level],
        )
    ]
```

### 2.3 YAML 配置文件示例

```yaml
# config.yaml
camera_name: front_camera
serial_no: '123456789'
enable_color: true
enable_depth: true
rgb_camera:
  color_profile: '1280,720,30'
  color_format: 'RGB8'
depth_module:
  depth_profile: '848,480,30'
  depth_format: 'Z16'
align_depth:
  enable: true
```

## 3. 多相机 Launch

### 3.1 rs_multi_camera_launch.py

```python
# 为每个相机创建独立的命名空间
# 通过 serial_no 区分设备
Node(
    namespace='camera1',
    name='camera1',
    parameters=[{'serial_no': '_123456'}],
)
Node(
    namespace='camera2',
    name='camera2',
    parameters=[{'serial_no': '_789012'}],
)
```

### 3.2 多相机同步

```yaml
# 同步模式
depth_module.inter_cam_sync_mode: 1  # Master
# 或
depth_module.inter_cam_sync_mode: 2  # Slave
```

## 4. 命令行使用示例

```bash
# 基本启动
ros2 launch realsense2_camera rs_launch.py

# 指定设备
ros2 launch realsense2_camera rs_launch.py serial_no:=_123456789

# 启用多种流
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    enable_infra1:=true \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=2

# 启用滤波器和点云
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    pointcloud.enable:=true \
    spatial_filter.enable:=true

# 使用配置文件
ros2 launch realsense2_camera rs_launch.py config_file:=/path/to/config.yaml

# 播放 rosbag
ros2 launch realsense2_camera rs_launch.py \
    rosbag_filename:=/path/to/file.bag \
    rosbag_loop:=true

# 指定分辨率
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:=1920,1080,30 \
    depth_module.depth_profile:=1280,720,30
```

## 5. Lifecycle Node 配置

通过 CMake 选项在编译时决定：

```cmake
option(USE_LIFECYCLE_NODE "Enable lifecycle nodes" OFF)

if(USE_LIFECYCLE_NODE)
    file(WRITE "global_settings.yaml" "use_lifecycle_node: true\n")
else()
    file(WRITE "global_settings.yaml" "use_lifecycle_node: false\n")
endif()
```

Launch 文件在运行时读取此配置选择节点类型。

## 6. 对 HAL 设计的启示

### 6.1 配置层抽象

```
DeviceConfig:
  + serialNumber: string
  + usbPort: string
  + deviceType: string
  + jsonFilePath: string
  
StreamConfig:
  + streams: map<StreamType, StreamSettings>
  
StreamSettings:
  + enabled: bool
  + width: int (0=default)
  + height: int (0=default)
  + fps: int (0=default)
  + format: string

FilterConfig:
  + filters: map<string, FilterSettings>
  
FilterSettings:
  + enabled: bool
  + options: map<string, any>

HALConfig:
  + device: DeviceConfig
  + streams: StreamConfig
  + filters: FilterConfig
  + imuConfig: IMUConfig
  + publishTF: bool
```

### 6.2 关键设计决策
1. **参数优先级**：命令行 > YAML文件 > 默认值
2. **Profile 字符串格式**：`"width,height,fps"` 简洁但不够灵活
3. **多相机通过命名空间隔离**：简单有效
4. **Lifecycle 编译时决定**：可以改为运行时配置
