# RealSense ROS 4.57.7 源码分析 - 滤波器 Pipeline

## 1. 概述

滤波器 Pipeline 是 realsense-ros 的后处理系统，负责对原始帧数据进行各种变换和增强。

### 文件位置
- `include/named_filter.h` / `src/named_filter.cpp` - 基类
- `include/pointcloud_filter.h` / `src/pointcloud_filter.cpp` - 点云滤波器
- `include/align_depth_filter.h` / `src/align_depth_filter.cpp` - 深度对齐滤波器

## 2. 滤波器类层次

```
NamedFilter (基类，包装 rs2::filter)
  ├── PointcloudFilter    (点云生成 + 发布)
  └── AlignDepthFilter    (深度到彩色的对齐)
```

## 3. NamedFilter 基类

```cpp
class NamedFilter {
protected:
    std::shared_ptr<rs2::filter> _filter;     // SDK 滤波器对象
    SensorParams _params;                      // 参数管理
    rclcpp::Logger _logger;
    bool _is_enabled;                          // 使能状态
    std::vector<std::string> _parameters_names;

public:
    NamedFilter(shared_ptr<rs2::filter> filter, 
                shared_ptr<Parameters> parameters, 
                rclcpp::Logger logger,
                bool is_enabled = false,
                bool register_options = true);
    
    rs2::frameset process(rs2::frameset frameset);  // 处理帧
    bool is_enabled() const { return _is_enabled; }
    void setParameters(function<void(const Parameter&)> enable_callback);
};
```

### 参数注册

```cpp
void NamedFilter::setParameters(enable_callback) {
    // 获取滤波器名称 (如 "decimation_filter", "spatial_filter")
    string module_name = rs2_to_ros(filter->get_info(RS2_CAMERA_INFO_NAME));
    
    // 注册使能参数: {filter_name}.enable
    string param_name = module_name + ".enable";
    _params.getParameters()->setParamT(param_name, _is_enabled, 
        [this, enable_callback](const Parameter& param) {
            _is_enabled = param.get_value<bool>();
            if (enable_callback) enable_callback(param);
        });
    
    // 注册滤波器特定选项 (exposure, magnitude 等)
    _params.registerDynamicOptions(*_filter, module_name);
}
```

## 4. 滤波器管道初始化 (`setupFilters()`)

```cpp
void BaseRealSenseNode::setupFilters() {
    // === 标准 RS2 滤波器（按处理顺序排列）===
    _filters.push_back(NamedFilter(make_shared<rs2::decimation_filter>()));      // 1. 抽取
    _filters.push_back(NamedFilter(make_shared<rs2::hdr_merge>()));              // 2. HDR 合并
    _filters.push_back(NamedFilter(make_shared<rs2::sequence_id_filter>()));     // 3. 序列 ID 过滤
    _filters.push_back(NamedFilter(make_shared<rs2::disparity_transform>()));    // 4. 深度→视差
    _filters.push_back(NamedFilter(make_shared<rs2::spatial_filter>()));         // 5. 空间滤波
    _filters.push_back(NamedFilter(make_shared<rs2::temporal_filter>()));        // 6. 时间滤波
    _filters.push_back(NamedFilter(make_shared<rs2::hole_filling_filter>()));    // 7. 孔洞填充
    _filters.push_back(NamedFilter(make_shared<rs2::disparity_transform>(false)));// 8. 视差→深度
    _filters.push_back(NamedFilter(make_shared<rs2::rotation_filter>()));        // 9. 旋转滤波
    
    // === 特殊滤波器 ===
    _pc_filter = PointcloudFilter(make_shared<rs2::pointcloud>());               // 10. 点云生成
    _filters.push_back(_pc_filter);
    
    _align_depth_filter = AlignDepthFilter(make_shared<rs2::align>(RS2_STREAM_COLOR)); // 11. 深度对齐
    _filters.push_back(_align_depth_filter);
    
    _colorizer_filter = NamedFilter(make_shared<rs2::colorizer>());              // 12. 深度着色
    _filters.push_back(_colorizer_filter);
}
```

### 处理顺序说明

```
原始帧 (depth/color/infrared)
  │
  ├── [1] decimation_filter     - 降低分辨率（加速后续处理）
  ├── [2] hdr_merge             - 合并 HDR 序列帧
  ├── [3] sequence_id_filter    - HDR 序列过滤
  ├── [4] disparity_transform   - depth → disparity（深度→视差）
  ├── [5] spatial_filter        - 空间域滤波（在视差空间中效果更好）
  ├── [6] temporal_filter       - 时间域滤波
  ├── [7] hole_filling_filter   - 孔洞填充
  ├── [8] disparity_transform(false) - disparity → depth（视差→深度）
  ├── [9] rotation_filter       - 图像旋转（0°/90°/-90°/180°）
  ├── [10] pointcloud           - 点云生成（需要原始深度）
  ├── [11] align_depth          - 深度对齐到彩色（在点云之后）
  └── [12] colorizer            - 深度着色（在对齐之后）
```

**关键**：点云在对齐之前处理，因为点云需要原始深度；着色在对齐之后处理，获得对齐后的着色深度图。

## 5. PointcloudFilter 详解

### 5.1 额外参数

```
pointcloud.enable                → bool (使能点云生成)
pointcloud.allow_no_texture_points → bool (是否包含无纹理点)
pointcloud.ordered_pc            → bool (是否保持有序点云)
pointcloud.stream_filter         → int  (纹理源: 2=color, 3=infrared)
pointcloud.stream_index_filter   → int  (纹理源索引)
pointcloud.pointcloud_qos        → string (QoS 策略)
```

### 5.2 点云发布 (`Publish()`)

```
PointcloudFilter::Publish(pc, time, frameset, frame_id)
  │
  ├── 检查订阅者
  │
  ├── 获取纹理源帧
  │     ├── RS2_FORMAT_RGB8 → "rgb" 颜色字段
  │     ├── RS2_FORMAT_Y8   → "intensity" 灰度字段
  │     └── RS2_FORMAT_Z16  → 无颜色字段
  │
  ├── 创建 PointCloud2 消息
  │     ├── fields: x, y, z [+ rgb/intensity]
  │     ├── width/height: 取决于 ordered_pc
  │     └── is_dense: ordered=false, unordered=true
  │
  ├── 填充点数据
  │     对每个 vertex:
  │       ├── 检查有效性 (z > 0 && valid_texture)
  │       ├── 写入 x, y, z
  │       └── 写入颜色 (reverse_memcpy: RGB → BGR)
  │
  └── 发布到 ~/depth/color/points
```

### 5.3 点云 QoS 管理

点云发布器根据使能状态动态创建/销毁：
```cpp
void setPublisher() {
    if (_is_enabled && !_pointcloud_publisher) {
        _pointcloud_publisher = create_publisher("~/depth/color/points", qos);
    } else if (!_is_enabled && _pointcloud_publisher) {
        _pointcloud_publisher.reset();  // 禁用时释放
    }
}
```

## 6. AlignDepthFilter 详解

深度对齐将深度图变换到彩色相机的视角，使每个深度像素与彩色像素对齐。

```cpp
class AlignDepthFilter : public NamedFilter {
    // 特殊回调: 使能/禁用时触发 Profile 变更
    // 因为对齐会影响额外的话题（aligned_depth_to_color）
    function<void(const Parameter&)> _update_align_depth_func;
};
```

对齐启用后的额外话题：
```
~/aligned_depth_to_color/image_raw      (对齐后的深度图)
~/aligned_depth_to_color/camera_info    (使用彩色相机的内参)
```

## 7. 滤波器参数完整列表

| 滤波器 | 参数名 | 类型 | 说明 |
|--------|--------|------|------|
| decimation | decimation_filter.enable | bool | 使能 |
| | decimation_filter.filter_magnitude | int | 降采样倍率 (2/4/8) |
| spatial | spatial_filter.enable | bool | 使能 |
| | spatial_filter.filter_magnitude | int | 迭代次数 |
| | spatial_filter.filter_smooth_alpha | double | 平滑系数 |
| | spatial_filter.filter_smooth_delta | double | 差异阈值 |
| temporal | temporal_filter.enable | bool | 使能 |
| | temporal_filter.filter_smooth_alpha | double | 平滑系数 |
| | temporal_filter.filter_smooth_delta | double | 差异阈值 |
| hole_filling | hole_filling_filter.enable | bool | 使能 |
| | hole_filling_filter.holes_fill | int | 填充模式 |
| disparity | disparity_filter.enable | bool | 使能 |
| hdr_merge | hdr_merge.enable | bool | 使能 |
| rotation | rotation_filter.enable | bool | 使能 |
| | rotation_filter.rotation | double | 旋转角度 |
| align_depth | align_depth.enable | bool | 使能 |
| colorizer | colorizer.enable | bool | 使能 |
| | colorizer.color_scheme | int | 着色方案 |
| pointcloud | pointcloud.enable | bool | 使能 |
| | pointcloud.stream_filter | int | 纹理源 |
| | pointcloud.ordered_pc | bool | 有序点云 |
| | pointcloud.allow_no_texture_points | bool | 包含无纹理点 |

## 8. GPU 加速 (GLSL)

当编译选项 `BUILD_ACCELERATE_GPU_WITH_GLSL` 开启时：

```cpp
#if defined (ACCELERATE_GPU_WITH_GLSL)
    _colorizer_filter = NamedFilter(make_shared<rs2::gl::colorizer>());
    _pc_filter = PointcloudFilter(make_shared<rs2::gl::pointcloud>());
#else
    _colorizer_filter = NamedFilter(make_shared<rs2::colorizer>());
    _pc_filter = PointcloudFilter(make_shared<rs2::pointcloud>());
#endif
```

GPU 加速的着色器和点云生成使用 OpenGL GLSL，需要：
- OpenGL 上下文 (GLFWwindow)
- GLFW 事件循环

## 9. 对 HAL 设计的启示

### 9.1 滤波器管道接口

```
IFilter:
  + getName() → string
  + isEnabled() → bool
  + setEnabled(enabled: bool)
  + process(frame: Frame) → Frame
  + getOptions() → vector<FilterOption>
  + setOption(name, value)

IFilterPipeline:
  + addFilter(filter: IFilter, position: int = -1)
  + removeFilter(filter: IFilter)
  + getFilters() → vector<IFilter>
  + process(frameset: FrameSet) → FrameSet
  + setEnabled(filter_name: string, enabled: bool)
```

### 9.2 关键设计决策
1. **链式处理**：滤波器顺序很重要（视差空间滤波效果更好）
2. **全部可配置**：每个滤波器独立使能/禁用/配置
3. **延迟创建**：点云发布器只在使能时创建
4. **GPU 可选**：通过编译选项切换 CPU/GPU 实现
5. **管道扩展性**：新滤波器只需实现 `NamedFilter` 接口
