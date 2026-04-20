# 12 — Launch 与配置系统对比

> **本章回答**:两家如何组织 Launch 文件?YAML 配置如何合并参数?多设备模板如何复用?HAL Adapter 的 Launch 层应该怎么设计?

---

## 1. Orbbec 侧:按设备系列分 Launch + ComposableNode

### 1.1 Launch 文件列表

| 文件 | 覆盖设备 |
|---|---|
| `gemini_330_series.launch.py` | Gemini 330/335/336/335L/335Le |
| `gemini2.launch.py` | Gemini 2 |
| `gemini2L.launch.py` | Gemini 2L |
| `femto_bolt.launch.py` | Femto Bolt |
| `femto_mega.launch.py` | Femto Mega / Mega I |
| `astra2.launch.py` | Astra 2 |
| `dabai_max.launch.py` | Dabai Max 系列 |
| `lidar.launch.py` | Pulsar ME450 / SL450 LiDAR |
| `multi_camera.launch.py` | 多相机复合 |

每台设备系列都有**独立** launch(因为默认参数、支持的流、特殊 option 不同)。

### 1.2 通用结构

```python
def generate_launch_description():
    # 1. 声明所有参数 (100+ 条 DeclareLaunchArgument)
    args = [
        DeclareLaunchArgument('camera_name',  default_value='camera'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('enable_color',  default_value='true'),
        DeclareLaunchArgument('color_width',   default_value='1280'),
        DeclareLaunchArgument('color_height',  default_value='720'),
        DeclareLaunchArgument('color_fps',     default_value='30'),
        # ... 上百条
    ]

    def launch_setup(context):
        params = load_parameters(context, args)

        node = ComposableNode(
            package='orbbec_camera',
            plugin='orbbec_camera::OBCameraNodeDriver',
            name=params['camera_name'],
            namespace=params['camera_name'],
            parameters=[params],
        )

        container = ComposableNodeContainer(
            name='camera_container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[node],
        )
        return [container]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
```

### 1.3 参数合并(`load_parameters`)

```python
def load_parameters(context, args):
    default_params = {arg.name: LaunchConfiguration(arg.name).perform(context) for arg in args}

    # 如果指定了 config_file_path,合并 YAML
    if config_file_path:
        yaml_params = load_yaml(config_file_path)
        default_params = merge_params(default_params, yaml_params)

    # skip_convert 保留字符串类型(不被误转换)
    skip_convert = {'config_file_path', 'usb_port', 'serial_number'}

    # 类型转换:"true"→True, "1"→1, "3.14"→3.14
    return {k: (convert_value(v) if k not in skip_convert else v)
            for k, v in default_params.items()}
```

### 1.4 多相机 Launch (`multi_camera.launch.py`)

```python
camera_1 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource('gemini_330_series.launch.py'),
    launch_arguments={'camera_name': 'camera_1', 'serial_number': 'AAA111'}.items(),
)
camera_2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource('gemini_330_series.launch.py'),
    launch_arguments={'camera_name': 'camera_2', 'serial_number': 'BBB222'}.items(),
)

return LaunchDescription([camera_1, camera_2])
```

每个相机独立进程(独立 ComposableNodeContainer)。

### 1.5 YAML 模板(节选)

```yaml
# config/camera_config.yaml
/camera:
  ros__parameters:
    serial_number: "AY3K41100C3"
    enable_color: true
    color_width: 1280
    color_height: 720
    color_fps: 30
    color_format: "MJPG"
    enable_depth: true
    depth_width: 848
    depth_height: 480
    depth_fps: 30
    depth_format: "Y16"
    enable_ir: false
    enable_point_cloud: true
    enable_colored_point_cloud: true
    align_mode: "HW"
    device_preset: "Default"
    enable_spatial_filter: false
    enable_temporal_filter: false
    enable_hole_filling_filter: false
    spatial_filter_alpha: 0.5
    spatial_filter_diff_threshold: 120
    # ...
```

---

## 2. RealSense 侧:主 Launch + intra-process + multi-sync

### 2.1 Launch 文件列表

| 文件 | 用途 |
|---|---|
| `rs_launch.py` | 单相机主启动 |
| `rs_multi_camera_launch.py` | 多相机独立进程 |
| `rs_multi_camera_launch_sync.py` | 多相机 + 硬件同步 |
| `rs_intra_process_demo_launch.py` | Intra-process 零拷贝演示 |

**单 Launch 覆盖所有设备**(D435/D455/D457/D585S/D555/T265)。

### 2.2 参数完整列表(节选)

```python
configurable_parameters = [
    # 设备
    {'name': 'camera_name',     'default': 'camera'},
    {'name': 'camera_namespace', 'default': 'camera'},
    {'name': 'serial_no',       'default': "''"},
    {'name': 'usb_port_id',     'default': "''"},
    {'name': 'device_type',     'default': "''"},              # 正则
    {'name': 'config_file',     'default': "''"},
    {'name': 'json_file_path',  'default': "''"},              # 高级模式 JSON
    {'name': 'initial_reset',   'default': 'false'},
    {'name': 'rosbag_filename', 'default': "''"},
    {'name': 'rosbag_loop',     'default': 'false'},
    {'name': 'accelerate_gpu_with_glsl', 'default': 'false'},
    {'name': 'log_level',       'default': 'info'},

    # 彩色
    {'name': 'enable_color',    'default': 'true'},
    {'name': 'rgb_camera.color_profile', 'default': '0,0,0'},
    {'name': 'rgb_camera.color_format',  'default': 'RGB8'},
    {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true'},

    # 深度
    {'name': 'enable_depth',    'default': 'true'},
    {'name': 'depth_module.depth_profile', 'default': '0,0,0'},
    {'name': 'depth_module.depth_format',  'default': 'Z16'},
    {'name': 'depth_module.exposure',      'default': '8500'},
    {'name': 'depth_module.gain',          'default': '16'},
    {'name': 'depth_module.enable_auto_exposure', 'default': 'true'},

    # 红外
    {'name': 'enable_infra',    'default': 'false'},
    {'name': 'enable_infra1',   'default': 'false'},
    {'name': 'enable_infra2',   'default': 'false'},

    # HDR
    {'name': 'depth_module.hdr_enabled', 'default': 'false'},
    {'name': 'depth_module.exposure.1',  'default': '7500'},
    {'name': 'depth_module.exposure.2',  'default': '1'},

    # IMU
    {'name': 'enable_gyro',       'default': 'false'},
    {'name': 'enable_accel',      'default': 'false'},
    {'name': 'enable_motion',     'default': 'false'},
    {'name': 'gyro_fps',          'default': '0'},
    {'name': 'accel_fps',         'default': '0'},
    {'name': 'unite_imu_method',  'default': '0'},

    # 同步与组合
    {'name': 'enable_sync',       'default': 'false'},
    {'name': 'enable_rgbd',       'default': 'false'},
    {'name': 'depth_module.inter_cam_sync_mode', 'default': '0'},

    # 对齐
    {'name': 'align_depth.enable', 'default': 'false'},

    # 其他
    {'name': 'enable_safety',  'default': 'false'},
    {'name': 'publish_tf',     'default': 'true'},
    {'name': 'tf_publish_rate','default': '0.0'},
    {'name': 'diagnostics_period', 'default': '0.0'},
]
```

### 2.3 YAML 覆盖

```python
# 支持用户提供 config_file 覆盖默认
if config_file:
    yaml_params = yaml.load(open(config_file))
    # merge:yaml 优先级 > cli > default
```

### 2.4 多相机同步 Launch

```python
cameras = [
    {'serial_no': 'SN1', 'inter_cam_sync_mode': 1},  # Master
    {'serial_no': 'SN2', 'inter_cam_sync_mode': 2},  # Slave
    {'serial_no': 'SN3', 'inter_cam_sync_mode': 2},  # Slave
]
for cam in cameras:
    include(rs_launch, launch_arguments=cam)
```

### 2.5 Intra-process demo

```python
container = ComposableNodeContainer(
    name='camera_container',
    composable_node_descriptions=[
        ComposableNode(
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            extra_arguments=[{'use_intra_process_comms': True}]),
        ComposableNode(  # 订阅者也在同容器才能零拷贝
            package='your_consumer', plugin='YourSub',
            extra_arguments=[{'use_intra_process_comms': True}]),
    ])
```

---

## 3. 对比总览

| 维度 | Orbbec | RealSense |
|---|---|---|
| Launch 文件数 | **9+**(按设备系列分) | 4 |
| 单 Launch 覆盖范围 | 1 个系列 | 全部设备 |
| 参数声明数 | 100+ per launch | 80+ |
| YAML 合并 | 支持,位于 `load_parameters` | 支持,原生 ROS2 方式 |
| 多相机同步 Launch | ❌(独立进程,无同步 launch 模板) | **✅** 专门 launch |
| ComposableNode | ✅(默认) | ✅(但 demo 专门) |
| Intra-process | 通过 `use_intra_process_comms` 参数 | 专门 demo |
| 类型转换逻辑 | Wrapper 自定义 `convert_value` | ROS2 原生 |
| Rosbag 支持 | ❌ | `rosbag_filename` + `rosbag_loop` |

### ⚠️ 关键差异

1. **单 Launch vs 多 Launch**:
   - RealSense 单 Launch 覆盖全,靠运行时参数(`device_type`)区分。
   - Orbbec 每个设备系列独立 Launch,因为默认值差异太大(比如 Gemini 330 默认 MJPG,Astra 2 默认 Y16)。
   - **HAL 决策**:采用"基础 Launch + 设备特定 YAML 默认覆盖" 模式——一个 Launch,多个默认 YAML。

2. **Rosbag 回放支持**:
   - RealSense **将 rosbag 当作一种"虚拟设备"**,通过 `rs_context` 打开 bag 文件作为 `rs2::device`。这是 SDK 级功能。
   - Orbbec 无。
   - **HAL 决策**:`IDeviceManager::create("realsense")` 可以额外支持 `create("rosbag", bag_path)` 模式。

---

## 4. 对 HAL Adapter 的 Launch 设计建议

### 4.1 Launch 分层

```
launch/
  camera_hal.launch.py              # 主通用 launch(单相机)
  multi_camera.launch.py            # 多相机
  multi_camera_sync.launch.py       # 多相机同步
  intra_process_demo.launch.py      # 低延迟示例

config/
  default.yaml                      # 通用默认
  devices/
    gemini_330.yaml                 # 设备系列默认
    gemini_2.yaml
    realsense_d435.yaml
    realsense_d455.yaml
    realsense_d585s.yaml
    orbbec_lidar.yaml
  profiles/
    high_quality.yaml               # 场景预设
    low_latency.yaml
    indoor_scan.yaml
```

用户命令:
```bash
ros2 launch camera_hal camera_hal.launch.py \
    backend:=realsense \
    device_config:=realsense_d455.yaml \
    profile:=low_latency.yaml
```

### 4.2 参数优先级

```
命令行 args > 用户 YAML (config_file) > 场景 profile YAML > 设备默认 YAML > default.yaml > 代码硬编码
```

### 4.3 通用参数列表(规范)

```yaml
# default.yaml(节选)
/**:
  ros__parameters:
    # HAL 后端选择
    backend: "auto"            # auto / orbbec / realsense

    # 设备匹配
    device_filter:
      serial_number: ""
      usb_port: ""
      ip_address: ""
      device_type_regex: ""
      wait_timeout_ms: -1

    # 进程锁
    enable_process_lock: false

    # Frame 选项
    camera_name: "camera"
    base_frame_id: "camera_link"
    tf_prefix: ""
    publish_tf: true
    tf_publish_rate: 0.0

    # 时间戳
    time_domain: "system"      # hardware / system / global

    # 流使能(通用)
    enable_depth: true
    enable_color: true
    enable_infra1: false
    enable_infra2: false
    enable_accel: false
    enable_gyro: false

    # Profile(通用)
    depth_module:
      depth_profile: "0,0,0"
      depth_format: "Z16"
    rgb_camera:
      color_profile: "0,0,0"
      color_format: "RGB8"

    # 对齐/RGBD
    align_depth:
      enable: false
    enable_rgbd: false

    # IMU
    unite_imu_method: 0
    linear_accel_cov: 0.01
    angular_velocity_cov: 0.01

    # 同步
    sync_mode: "free_run"      # free_run / primary / secondary / secondary_synced / software_trigger / hardware_trigger

    # 诊断
    diagnostics_period: 1.0
```

### 4.4 设备特定 YAML(示例)

```yaml
# devices/realsense_d455.yaml(仅覆盖与 default 不同的项)
/**:
  ros__parameters:
    backend: "realsense"
    depth_module:
      depth_profile: "848,480,30"
    rgb_camera:
      color_profile: "1280,720,30"
    enable_infra1: true
    enable_infra2: true
    unite_imu_method: 2        # LINEAR_INTERPOLATION(D455 默认行为)
```

### 4.5 Launch 代码骨架

```python
def generate_launch_description():
    # 1. 通用 args(所有设备共用)
    args = declare_common_args()

    def launch_setup(context):
        backend = LaunchConfiguration('backend').perform(context)

        # 2. 合并:default → device_config → profile → user_config
        params = load_layered_params(context)

        # 3. 创建 Composable Node
        return [ComposableNodeContainer(
            ...,
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_hal_ros',
                    plugin='camera_hal::HalCameraDriver',
                    name=params['camera_name'],
                    namespace=params['camera_name'],
                    parameters=[params],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )])]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
```

### 4.6 多相机 Launch 模板

```python
def generate_launch_description():
    cameras = [
        {'name': 'camera_1', 'serial_no': 'AAA', 'sync_mode': 'primary'},
        {'name': 'camera_2', 'serial_no': 'BBB', 'sync_mode': 'secondary_synced'},
        {'name': 'camera_3', 'serial_no': 'CCC', 'sync_mode': 'secondary_synced'},
    ]

    container = ComposableNodeContainer(name='multi_camera_container', ...)

    for cam in cameras:
        container.add_composable_node(
            ComposableNode(
                package='camera_hal_ros',
                plugin='camera_hal::HalCameraDriver',
                name=cam['name'],
                namespace=cam['name'],
                parameters=[cam, ...],
            ))
    return LaunchDescription([container])
```

### 4.7 Rosbag 回放模式

```python
# rosbag_playback.launch.py
ComposableNode(
    package='camera_hal_ros',
    plugin='camera_hal::HalCameraDriver',
    parameters=[{'backend': 'rosbag',
                  'rosbag_path': '/path/to/bag.db3',
                  'rosbag_loop': True,
                  ...}])
```

HAL `IDeviceManager::create("rosbag", path)` 返回一个模拟设备。

### 4.8 坑

- ⚠️ **参数类型转换**:ROS2 Python Launch 的类型推断不完美,serial 等纯字符串需要显式标记 `TextSubstitution` 防止转为 int。
- ⚠️ **命名冲突**:多相机时每个相机必须有独立 namespace,否则参数冲突(特别是 `camera_name` 本身)。
- ⚠️ **Composable Node 的参数传递**:`extra_arguments` 与 `parameters` 不同,`use_intra_process_comms` 必须在前者。
- ⚠️ **Lifecycle 启动延迟**:Lifecycle Node 在 launch 启动后不会自动 configure/activate,需要额外的 `LifecycleNode` transition 或 `lifecycle_manager`。Adapter 应提供简单模式。
