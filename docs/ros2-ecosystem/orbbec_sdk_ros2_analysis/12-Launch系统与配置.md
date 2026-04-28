# Launch 系统与配置分析

## 1. Launch 文件列表

| 文件名 | 设备系列 |
|--------|---------|
| gemini_330_series.launch.py | Gemini 330/335/336/335L/335Le 等 |
| gemini2.launch.py | Gemini 2 |
| gemini2L.launch.py | Gemini 2L |
| femto_bolt.launch.py | Femto Bolt |
| femto_mega.launch.py | Femto Mega / Mega I |
| astra2.launch.py | Astra 2 |
| dabai_max.launch.py | Dabai Max 系列 |
| lidar.launch.py | LiDAR 设备 |
| multi_camera.launch.py | 多相机 |

---

## 2. Launch 文件通用结构

```python
def generate_launch_description():
    # 1. 声明所有参数（100+ DeclareLaunchArgument）
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('enable_color', default_value='true'),
        # ...
    ]

    def launch_setup(context):
        # 2. 加载并合并参数
        params = load_parameters(context, args)

        # 3. 创建 ComposableNode
        node = ComposableNode(
            package='orbbec_camera',
            plugin='orbbec_camera::OBCameraNodeDriver',
            name=params['camera_name'],
            namespace=params['camera_name'],
            parameters=[params],
        )

        # 4. 放入 ComposableNodeContainer
        container = ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[node],
        )
        return [container]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
```

---

## 3. 参数加载函数

```python
def load_parameters(context, args):
    # 1. 收集 Launch 默认参数
    default_params = {arg.name: LaunchConfiguration(arg.name).perform(context)
                      for arg in args}

    # 2. 加载 YAML 文件并覆盖（如果指定了 config_file_path）
    if config_file_path:
        yaml_params = load_yaml(config_file_path)
        default_params = merge_params(default_params, yaml_params)

    # 3. 跳过转换的特殊参数（usb_port, serial_number 等保持字符串）
    skip_convert = {'config_file_path', 'usb_port', 'serial_number'}

    # 4. 类型转换（"true"→True, "1"→1, "3.14"→3.14）
    result = {k: convert_value(v) if k not in skip_convert else v
              for k, v in default_params.items()}
    return result
```

---

## 4. YAML 配置系统

### 4.1 common.yaml 统一配置入口（v2.7.x 新增）

```yaml
orbbec_ros:
  camera_parameters:
    general:
      camera_model: "gemini330_series"
      config_file_path: "gemini330_series.yaml"
      log_level: "none"

    device:
      camera_name: "camera"
      serial_number: ""
      usb_port: ""
      device_num: 1
      time_domain: "device"
      enable_ldp: true

    color:
      enable_color: true
      color_width: 0
      color_height: 0
      color_fps: 0
      color_format: "ANY"
      enable_color_auto_exposure: true

    depth:
      enable_depth: true
      depth_registration: true
      align_mode: "SW"
      depth_work_mode: ""

    ir:
      enable_ir: false

    point_cloud:
      enable_point_cloud: true
      enable_colored_point_cloud: false

    imu:
      enable_accel: false
      enable_gyro: false

    post_processing:
      depth_filter:
        enable_noise_removal_filter: true
        enable_spatial_filter: true

    multi_device_sync:
      sync_mode: ""

    tf:
      publish_tf: true
      tf_publish_rate: 0
```

### 4.2 设备特定 YAML
每个设备系列有独立的配置文件（`gemini330_series.yaml`、`gemini2.yaml` 等），覆盖设备特有的默认值。

### 4.3 深度滤波器配置
`config/depthfilter/` 目录下存放设备专用的滤波器配置文件，通过 `depth_filter_config` 参数指定路径。

---

## 5. 参数优先级

```
命令行参数（ros2 launch ... key:=value）  ← 最高优先级
    │
    ▼
Launch 文件 DeclareLaunchArgument default_value
    │
    ▼
config_file_path 指定的 YAML 文件
    │
    ▼
代码默认值（constants.h）               ← 最低优先级
```

---

## 6. 多相机 Launch 配置

```python
cameras = [
    {'camera_name': 'camera_1', 'serial_number': 'AAA', 'usb_port': '2-3.1'},
    {'camera_name': 'camera_2', 'serial_number': 'BBB', 'usb_port': '2-3.2'},
]

actions = []
for cam in cameras:
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('gemini_330_series.launch.py'),
            launch_arguments=cam.items(),
        )
    )

return LaunchDescription(actions)
```

---

## 7. 对 HAL 的配置设计建议

```yaml
# hal_config.yaml
hal_camera:
  driver: "orbbec"

  device:
    selector:
      serial_number: "ABC123"

  streams:
    - type: color
      enabled: true
      resolution: [640, 480]
      fps: 30
    - type: depth
      enabled: true
      resolution: [640, 480]
      fps: 30

  processing:
    filters:
      - name: noise_removal
        enabled: true
        params: {min_diff: 256, max_size: 80}
      - name: spatial
        enabled: true
        params: {alpha: 0.5}

  sync:
    mode: "free_run"
    time_domain: "device"
```
