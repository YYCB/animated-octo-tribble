# 16 — HAL 实施路线图与迁移策略

> **本章目的**:给出从现在到 HAL v1.0 的可执行路线图、里程碑、迁移风险、以及对未来支持 ToF/单目/立体等新品类的扩展策略。

---

## 1. 六阶段路线图

### 阶段 0:准备(1~2 周)
**目标**:搭建仓库骨架,固定接口 API

交付物:
- `camera_hal/core/include/` 完整接口头文件(参见 [第 15 章](15-HAL核心接口详细设计.md))
- CMake 框架
- CI 基线(编译 + 静态检查)
- 接口评审通过(建议设计评审)

退出条件:
- HAL Core 头文件编译通过(无实现)
- 至少 3 位相机领域同事评审接口并签字
- 接口文档(docstring + 使用示例)齐备

### 阶段 1:Orbbec 后端 + 最小 ROS Adapter(3~4 周)
**目标**:跑通端到端,证明接口合理

交付物:
- `camera_hal/backend_orbbec/` 实现(至少覆盖 Gemini 330 USB)
- `camera_hal/ros_adapter/` 最小实现:
  - 发 color/depth/IR/camera_info/TF
  - 参数系统基础(静态参数)
- `launch/camera_hal.launch.py` 主 launch
- Orbbec 设备集成测试(至少 1 种真设备)

退出条件:
- Gemini 330 USB 接入,30fps 彩色+深度持续 1 小时无掉帧
- RViz 能看到正常的 depth / color / pointcloud
- 与 OrbbecSDK_ROS2 v2.7.6 的 topic 接口兼容度 ≥ 80%(topic 名/消息结构/TF)

### 阶段 2:RealSense 后端 + 参数自动映射(2~3 周)
**目标**:第二后端验证接口通用性,引入关键特色

交付物:
- `camera_hal/backend_realsense/` 实现(至少覆盖 D455)
- ROS Adapter 引入 **SDK Option → ROS Param 自动映射**(见 [第 5 章](05-参数系统对比.md) §2.3)
- `IParameterProvider` ROS2 实现(含异步更新线程、自反馈保护)
- `rs_launch.py` 相容层(帮助现有 realsense-ros 用户迁移)

退出条件:
- 同一份 launch 文件,通过 `backend` 参数可以切换 Orbbec/RealSense
- `ros2 param list` 能看到完整设备 option 列表
- D455 深度+彩色+IMU 同时运行,软件融合 IMU 输出到 `/camera/imu`

### 阶段 3:滤波器、多相机、同步(3~4 周)
**目标**:覆盖高级功能

交付物:
- `IFilterPipeline` 默认构造(每后端有自己的推荐管线)
- `NamedFilter` 风格自动注册滤波器参数
- Point cloud / AlignFilter 两个特化子类
- `ISyncManager` 在两个后端实现
- 多相机 Launch 模板 + 进程锁(Orbbec)
- 硬件同步集成测试(实机 2 台设备)

退出条件:
- `depth_module.spatial_filter.enable` 可动态 on/off
- 2 台 Gemini 330 硬件同步,帧间时间差 ≤ 1ms
- 滤波器处理后的深度点云在 RViz 中视觉上平滑

### 阶段 4:解码器插件 + Jetson/RK 加速(2~3 周)
**目标**:嵌入式性能达标

交付物:
- `camera_hal/decoders/libjpeg_turbo/` 软解
- `camera_hal/decoders/jetson_nv/` Jetson 硬解(条件编译)
- `camera_hal/decoders/rk_mpp/` RK 硬解(条件编译)
- Pluginlib 注册 + 运行时选择
- Jetson Orin NX 实测 MJPEG 1080p@30fps 编解码 CPU 占用 < 15%

退出条件:
- Jetson 平台 Gemini 335 1080p MJPEG 彩色+480p 深度同时 30fps,CPU < 50%
- 软解后端作 fallback,自动选择生效

### 阶段 5:特色扩展 + 生产就绪(持续)
**目标**:覆盖设备特有能力、生产稳定性

交付物:
- LiDAR 支持(Orbbec Pulsar)
- Safety 相机(RealSense D585S)
- 网络设备(Femto Mega I)
- Rosbag 后端(`IDeviceManager::create("rosbag", path)`)
- 固件升级 Action
- On-chip Calibration Action
- Diagnostic 标准化(`diagnostic_msgs`)
- 性能基线 + Nightly perf test
- 稳定性:72 小时压力测试

退出条件:
- HAL v1.0 发布
- 文档完整(API + 使用手册 + 迁移指南)
- 2 个真实项目试点落地

---

## 2. 里程碑时间线

```
Week  0───────────1──────────3──────────7──────────10─────────14────────20+
      │           │          │          │          │          │
      ├─ S0 准备 ─┤          │          │          │          │
      │           ├── S1 Orbbec ────────┤          │          │
      │                      ├── S2 RS + Params ───┤          │
      │                                 ├── S3 滤波/多相机 ───┤
      │                                            ├── S4 解码/Jetson ┤
      │                                                       ├── S5 特色扩展 → v1.0
```

总计 **~5 个月**(不含阶段 5 长尾),2-3 人全职。

---

## 3. 关键风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| **SDK ABI 变化**(OrbbecSDK 或 librealsense2 升级破坏 API) | 编译失败、运行时行为变化 | 在 Backend 层加版本检测 + 编译期守卫;CI 矩阵含多个 SDK 版本 |
| **接口设计过度抽象** | Core 难实现、后端写很累 | 阶段 1 用 Orbbec 验证,发现过度抽象立即回退;阶段 2 才加 RealSense |
| **性能回归**(HAL 引入拷贝/虚函数开销) | 彩色掉帧、CPU 占用高 | 零拷贝设计(`IFrame::getData` 返回 SDK 指针);perf test 作为 CI gate |
| **ROS2 Distro 不兼容** | Jazzy/Humble 行为差异 | ROS Adapter 层加 `#if RCLCPP_VERSION_*` 保护;Distro-specific CI |
| **Jetson CUDA 版本锁死** | 用户 CUDA 11 vs 12 编译失败 | CMake 自动检测,支持主流两个版本 |
| **设备固件升级中断** | 变砖风险 | Firmware update Action 明确警告 + 至少 3 次重试 + CRC 校验 |
| **多相机进程锁泄露** | 重启后设备抢占失败 | 用 `PTHREAD_MUTEX_ROBUST` + `pthread_mutex_consistent` |
| **Lifecycle 与非 Lifecycle 双模式维护成本** | 代码分叉 | 用 `RosNodeBase` 模板化(见 realsense-ros),单代码路径 |
| **法务/License 冲突**(OrbbecSDK 非开源 SDK + Apache-2 wrapper) | 分发风险 | HAL Core Apache-2;Backend 各自遵守 SDK license |

---

## 4. 与现有生态的迁移策略

### 4.1 对 OrbbecSDK_ROS2 用户

目标:**现有 launch 命令改一行就能跑 HAL 版本**

步骤:
1. 保留 topic/tf 命名完全兼容(`/camera/color/image_raw` 等)
2. 提供 `legacy_orbbec.launch.py` 把老的参数名映射到新名
3. 自定义 msg 包里 typedef 兼容 `orbbec_camera_msgs::msg::Extrinsics`(内容结构相同)
4. Service 端保留 Orbbec 40+ service 的 **读** 版本(只读设备状态),**写** 版本映射到 ROS Param 并输出 deprecation warning

### 4.2 对 realsense-ros 用户

目标:同样一份 launch 兼容

步骤:
1. 参数名完全照搬(`rgb_camera.color_profile`, `depth_module.*`)
2. 自定义 msg 包提供 `realsense2_camera_msgs` 的 shim(转发到 `camera_hal_msgs`)
3. Lifecycle 模式支持
4. Rosbag 虚拟设备支持

### 4.3 对 SDK 版本的兼容矩阵

| | OrbbecSDK v2.6 | v2.7.x | v2.8(未来) |
|---|---|---|---|
| HAL v1.0 | 不支持 | ✅ | 尽量兼容 |

| | librealsense 2.54 | 2.55 | 2.56 | 2.57 |
|---|---|---|---|---|
| HAL v1.0 | 不支持 | ✅ | ✅ | ✅ |

---

## 5. 扩展策略:新品类设备

### 5.1 单目 UVC 相机(如 USB 摄像头)

**结论**:不走 HAL。走 `image_transport` / `usb_cam` 即可。HAL 聚焦 3D/深度/多流相机。

### 5.2 立体相机(未集成 SDK 的纯 UVC 双目)

可作为 **`Backend::UVCStereo`**,从 V4L2 拉帧,手工做视差计算。HAL 接口已足以表达,但性价比低,不推荐优先。

### 5.3 ToF 相机(非 Orbbec/Intel)

- **Azure Kinect**:SDK 架构与 librealsense 相似,可作为 `backend_k4a/` 直接实现。
- **PMD/Espros**:SDK 更原始,需要更多 glue code,但接口天然覆盖。

### 5.4 Event Camera(DVS/Prophesee)

需扩展 `StreamType` 加 `Event`,`IFrame` 子类 `IEventFrame`。数据模型显著不同(事件流非帧),不建议塞进 HAL。

### 5.5 LiDAR(机械/固态)

已在 HAL 设计内:
- `SensorCategory::Lidar`
- 输出 `LaserScan` 或 `PointCloud2`(球坐标/笛卡尔)
- 可复用 `ISyncManager` 做 GPIO PPS 同步(`SyncMode::HardwareTrigger`)

### 5.6 Fisheye / Pose Tracker(T265 类)

已设计:
- `StreamType::Pose6DoF`
- `IPoseFrame` 子类

### 5.7 未来 DDS 原生相机(D555 趋势)

Backend 不走 SDK,走 DDS topic 订阅,但对外仍是 `IDevice`/`ISensor`。只是底层传输变。HAL 接口不变。

---

## 6. 测试策略

### 6.1 金字塔

```
┌─────────────────┐
│   E2E (硬件)    │ 10% — 实机 nightly
├─────────────────┤
│  集成(rosbag)  │ 30% — Rosbag 后端回放标定录像
├─────────────────┤
│  单元(Mock)    │ 60% — MockIDevice/MockISensor
└─────────────────┘
```

### 6.2 Rosbag 后端作为集成测试载体

- 录制"基准数据集":Gemini 330、D455 各录 1 小时 profile 多样数据。
- CI 每次跑:HAL + Rosbag 后端 + ROS Adapter → 验证 topic/frame/timestamp 正确。
- **关键价值**:没有硬件也能跑完整 pipeline 测试,开发者本地开发友好。

### 6.3 性能基准

```cpp
// tests/perf/test_frame_latency.cpp
BENCHMARK(FrameLatency_848x480_Z16_30fps) {
    // 从 SDK 帧回调到 ROS publish 的端到端延迟
    // 阈值: P95 < 15ms, P99 < 30ms
}

BENCHMARK(MjpegDecode_1920x1080) {
    // 软解 vs Jetson 硬解
    // 软解: > 60fps on x86, 硬解: > 90fps on Jetson
}

BENCHMARK(PointCloud_848x480) {
    // CPU 生成时间
    // 阈值: < 20ms/frame
}
```

### 6.4 稳定性测试

- **长耗时**:72 小时持续运行无 assert/crash;内存泄露 < 1MB/小时
- **热插拔**:脚本驱动 USB hub 每 10 秒拔插,观察 HAL 状态机恢复
- **多相机压力**:4 台 Gemini 330 + 2 台 D455 同时跑 12 小时

---

## 7. 文档交付清单

- **API Reference**(Doxygen 生成)
- **Getting Started**(5 分钟跑起来)
- **Device Support Matrix**(哪些设备支持哪些能力)
- **Migration Guide**:
  - from OrbbecSDK_ROS2
  - from realsense-ros
- **Backend Developer Guide**(怎么写新后端)
- **Filter Developer Guide**(怎么写新 filter)
- **Decoder Developer Guide**(怎么写新 decoder)
- **Troubleshooting**(常见问题)
- **Performance Tuning**(Jetson/RK 调优)

---

## 8. 最终建议

### 8.1 做什么

1. **先用 Orbbec 单后端验证接口**——别一开始就双后端,会陷入过度抽象。
2. **RealSense 的参数系统是金矿**——自动映射、异步更新、双向同步,整套搬过来。
3. **Orbbec 的 multi-sync + LiDAR + 网络设备是差异化价值**——HAL 必须能表达。
4. **用 pluginlib 做 Backend/Decoder 的动态加载**——比静态链接灵活多。
5. **Rosbag 后端优先级不低**——测试基础设施,越早越好。

### 8.2 不做什么

1. ❌ **不要追求 API 层的单一时钟域**——让 `IFrame::getTimestampDomain()` 报告真实情况,转换交给 Adapter 层。
2. ❌ **不要把 Lifecycle 做进 Core**——是 ROS 的事。
3. ❌ **不要把所有 SDK option 写进 HAL 接口常量**——用 `OptionInfo` 动态描述,SDK 升级免改。
4. ❌ **不要同时支持 ROS1 + ROS2**——ROS1 EOL,只做 ROS2。
5. ❌ **不要在 HAL 里写 CV 处理**(除了必要的解码和翻转)——那是 image_pipeline 的事。

### 8.3 成功标准

v1.0 发布时应满足:
- ✅ 至少 2 个 backend(orbbec + realsense)功能覆盖 > 90%
- ✅ 3 个真实机器人项目试点运行
- ✅ 性能无回归(vs 原厂 wrapper CPU/延迟/帧率)
- ✅ 至少 1 个 Jetson 平台测试通过
- ✅ API 冻结,后续 1.x 版本保持 ABI 兼容

---

**到此,完整的 HAL 详细设计输入已备齐。**
实施者应按本路线图逐阶段推进,每阶段末召开技术评审,将实际问题反馈回 [第 15 章](15-HAL核心接口详细设计.md) 的接口定义做微调,但避免大改大修。
