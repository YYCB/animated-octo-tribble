# 评估与基线 — 怎么客观判断 SLAM 栈"变好了"

> 关联：所有前序章节。本文是"调参 → 验证 → 留基线"的方法论闭环。

> **核心论点**：调 SLAM 不能靠肉眼看 RViz。必须用客观指标 + 可重复数据集，否则永远在"感觉好像变好了"循环里。本文给出在 S100 / ARM 平台上**纯本地、CPU-only** 的基线方法学。

---

## 一、为什么必须做基线

| 没基线 | 有基线 |
|--------|-------|
| "今天调了 `coarse_angle_resolution`，感觉走廊好像不漂了" | "今天 APE_RMSE 从 0.18m 降到 0.11m，回归测试通过" |
| 同事改了一行参数，没人发现地图退化 | CI 自动跑测试集，回归即报警 |
| 换硬件平台不知道损失多少精度 | Orin AGX vs S100 的 APE 差值有数 |
| 报问题时只能靠视频 | 提供 `evo_traj` 输出 + bag 文件 |

---

## 二、关键指标定义

### 2.1 APE（Absolute Pose Error）

> 评估**全局一致性**——估计轨迹与真值在同一坐标系下逐帧位置/姿态差异。

```
APE_t(k) = || t_est(k) − t_gt(k) ||₂
APE_R(k) = angle( R_est(k)ᵀ × R_gt(k) )
```

汇总统计：mean / median / RMSE / std / min / max。**RMSE 最常用**，对大误差敏感。

### 2.2 RPE（Relative Pose Error）

> 评估**短期里程计精度**——固定窗口（如 1s 或 1m）内的相对位姿差异。

```
RPE(k, Δ) = || (T_est(k+Δ) ⊖ T_est(k)) ⊖ (T_gt(k+Δ) ⊖ T_gt(k)) ||
```

RPE 不受全局漂移影响，专门衡量"局部抖不抖"。**SLAM Toolbox 调参主要看 RPE**；VIO 调参也看 RPE。

### 2.3 何时看哪个

| 指标 | 衡量什么 | 适合 |
|------|---------|------|
| APE_RMSE_t | 全局轨迹精度 | 完整跑完一圈后比较算法 |
| RPE_RMSE_t (Δ=1m) | 米级局部漂移 | 调 scan_matcher 参数 |
| RPE_RMSE_R (Δ=1s) | 秒级旋转抖动 | 调 IMU/EKF |
| Loop Closure Detection Rate | 回环触发次数 / GT 真实回环数 | 回环算法对比 |
| Map IoU vs GT 占据图 | 地图重叠度 | 整体建图质量 |

---

## 三、`evo` 工具链（标准）

> [`MichaelGrupp/evo`](https://github.com/MichaelGrupp/evo)，纯 Python，pip 可装

### 3.1 安装

```bash
pip install evo --upgrade --no-binary evo
```

### 3.2 输入格式

evo 接受多种格式，最常用：

| 格式 | 文件 | 说明 |
|------|------|------|
| TUM | `timestamp tx ty tz qx qy qz qw`（每行）| 最简单 |
| KITTI | 12 个浮点数（3×4 变换矩阵展平）| 不带时间 |
| EuRoC | CSV，每行带时间 | VIO 数据集标准 |
| **bag** | 直接 `evo_traj bag <file.bag> /odom` | 最方便 |

ROS 2 用户：直接喂 `.mcap` 或 `.db3` rosbag2 文件，evo 自带 reader。

### 3.3 典型命令

```bash
# 查看一条轨迹
evo_traj bag2 my_run.mcap /odom --plot

# 与 GT 对比 APE
evo_ape bag2 my_run.mcap /odom_gt /odometry/filtered --plot --plot_mode xy

# RPE
evo_rpe bag2 my_run.mcap /odom_gt /odometry/filtered \
    --delta 1.0 --delta_unit m --plot

# 多次运行结果对比（A/B 对照）
evo_res *.zip --use_filenames -p
```

### 3.4 三轨迹同图（调试 EKF 必备）

```bash
evo_traj bag2 my_run.mcap \
    /odom \                    # 纯 wheel
    /odometry/filtered \       # 融合后
    --ref /odom_gt \
    --plot --plot_mode xy
```

一眼能看出"融合到底有没有让 odom 变好"。

---

## 四、Ground Truth 怎么来（CPU-only 平台的难题）

S100 没有 NVIDIA 仿真栈（Isaac Sim），但 GT 来源仍然有多种可选：

### 4.1 仿真：Gazebo Harmonic（推荐）

> 关联：`docs/ros2-ecosystem/simulation_research/`

```
Gazebo Harmonic
  ├── 内置 IMU / LiDAR / Camera plugin
  ├── /model/<name>/pose 提供完美 GT
  ├── ros_gz_bridge 把 GT 转为 nav_msgs/Odometry → /odom_gt
  └── 全 CPU 仿真，S100/x86/Orin 都能跑
```

**最佳实践**：本地 x86 工作站跑 Gazebo + SLAM 栈，回归测试只在工作站做；S100 上做的是部署后的实地验证，不再做 GT 评估。

### 4.2 公开数据集回放

| 数据集 | 传感器 | GT 来源 | 适合 |
|--------|-------|---------|------|
| TUM RGB-D | RGB-D | Vicon | VIO / RGB-D SLAM |
| EuRoC MAV | stereo + IMU | Leica / Vicon | VIO |
| KITTI | LiDAR + camera | RTK-GNSS + IMU | LiDAR SLAM（户外）|
| Hilti SLAM | 多传感器 | TLS 静态扫描 | 室内多传感器 |
| Newer College | LiDAR + IMU | TLS | LiDAR + VIO |

**用 ROS 2 回放**：先转 bag → 用 `rosbag2` 回放 → 把待测算法当作"online 节点"接收。

### 4.3 实地：动捕 / Total Station / 二维码地标

- **OptiTrack / Vicon**：金标准，但贵且范围有限
- **Total Station 跟踪棱镜**：单点跟踪，分钟级精度，便宜
- **AprilTag 地面铺设**：在已知位置贴 ID 标签，机器人通过相机检测，**自标定 GT**——CPU 友好，S100 完全可用

---

## 五、回归测试套件（建议工程化落地）

```
tests/slam_regression/
├── datasets/
│   ├── short_corridor.mcap          # 长走廊，2 min
│   ├── office_loop.mcap             # 办公室一圈，8 min
│   ├── dynamic_lobby.mcap           # 大堂 + 行人，5 min
│   └── glass_walls.mcap             # 玻璃幕墙，3 min
├── configs/
│   ├── baseline.yaml                # 基线参数集
│   └── candidate.yaml               # 待测试参数集
├── run_regression.sh
└── expected/
    ├── baseline_metrics.json        # 基线 APE/RPE
    └── thresholds.json              # 允许的劣化范围
```

`run_regression.sh` 伪代码：

```bash
for dataset in datasets/*.mcap; do
    for config in configs/*.yaml; do
        # 1) 启动 SLAM 栈（headless）
        # 2) ros2 bag play $dataset
        # 3) ros2 bag record /odometry/filtered /odom_gt → out.mcap
        # 4) evo_ape → metrics_$config_$dataset.json
        # 5) compare with expected
    done
done
```

CI 集成：
- PR 改了 `slam_toolbox` 参数 → 触发 regression
- APE 劣化 > 10% → fail
- 通过 → 自动更新 baseline_metrics.json

---

## 六、性能（不只是精度）的基线方法学

### 6.1 三件套

| 维度 | 工具 | 触发频率 |
|------|------|---------|
| CPU 占用 / 核心 | `pidstat -u -p <PID> 1` 或 `htop` | 每个回归运行抓 1Hz 采样 |
| 内存 | `pidstat -r -p <PID> 1` | 同上 |
| 端到端延迟 | `ros2_tracing` + LTTng | 单独跑 latency 数据集 |

### 6.2 SLAM Toolbox 关键 Callback 延迟

```
关键路径：scan_callback → scan_matcher → mapper.process → graph optimize → publish /map

需要测：
  T1: /scan 到达 to scan_callback 入口（rmw 队列耗时）
  T2: scan_matcher 单帧耗时
  T3: mapper.process 完整一帧（含可能的回环 + optimize）
  T4: /map 发布到 ros2 topic
```

ros2_tracing 用例（节选）：
```bash
ros2 trace start slam_run
# ...运行 SLAM 一段时间...
ros2 trace stop
# 用 babeltrace2 / tracecompass 分析
```

> 详细 tracing 套件见 `realtime_research/`。本节只指出**必须跑一次 baseline tracing**，否则后续优化没基准。

### 6.3 S100 vs Orin AGX 对照表（建立基线模板）

> 用于跨平台移植决策：S100 主控时各项指标的可接受范围。

| 指标 | x86（开发机）| Orin AGX | **S100（待实测）** | 阈值 |
|------|------------|---------|-------------------|------|
| office_loop APE_RMSE_t | 0.08m | 0.10m | TBD | < 0.15m |
| office_loop RPE_RMSE_t (1m) | 0.02m | 0.025m | TBD | < 0.04m |
| SLAM Toolbox 单核占用 | 18% | 22% | TBD | < 60% |
| EKF 单核占用 | 4% | 5% | TBD | < 15% |
| STVL costmap 更新延迟 | 35ms | 40ms | TBD | < 100ms |
| 回环检测耗时（lifelong）p99 | 80ms | 100ms | TBD | < 500ms |
| `/map` 到 `/cmd_vel` 端到端延迟 | 250ms | 280ms | TBD | < 500ms |

**建立 S100 基线的标准步骤**：
1. 选定 baseline 参数集（[01](./01_slam_toolbox_deep_dive.md) §七 推荐起步参数）
2. 在 S100 上跑 `office_loop.mcap` 回归套件 5 次
3. 取中位数填入"S100 待实测"列
4. 后续每次硬件 / 参数 / 上游版本变更，对照该列检查

---

## 七、单数据集深度剖析（office_loop 示例）

```
轨迹：办公室一圈，8 分钟，含 1 次"经过同位置"以触发回环

Pass 1（baseline，[01] §七 推荐参数）：
  evo_ape: APE_RMSE_t = 0.115m
  evo_rpe (Δ=1m): RPE_RMSE_t = 0.024m
  loop closure detected: 1（次数与 GT 匹配）✅
  CPU avg: 32% / core
  memory: 280MB

Pass 2（开启 lifelong，5 分钟续跑）：
  CPU avg: 41% / core（+9%）
  vertex 数从 1240 增至 1410
  utility 裁剪了 80 个旧 vertex
  无错误回环

Pass 3（关闭 IMU，仅 wheel odom）：
  APE_RMSE_t = 0.245m  ← 暴露了 IMU 的价值
  RPE_RMSE_R = 0.031 rad ← 旋转误差是平移的 1.5×
  scan_matcher 失败次数 +6

Pass 4（IMU 协方差填错——模拟常见误用）：
  APE_RMSE_t = 0.175m  ← 比关 IMU 还差！
  ← 凸显 [02] §四 协方差填法的重要性
```

> 这种"参数对照表"是说服团队接受调参变更的最有力证据。

---

## 八、与 chassis_protocol 的协作

为了让评估可重复，建议 chassis_protocol 暴露：

| 接口 | 用途 |
|------|------|
| `/odom` 含真实协方差（不是 0/1e-9）| 让 EKF 评估有意义 |
| 一个 service `set_simulated_slip(bool)` | 注入打滑场景做鲁棒性测试 |
| `/joint_states` 与 `/odom` 时间戳一致 | tracing 时能对齐 |

> chassis_protocol 当前是否已实现这些可在源码层确认；列入 [07](./07_chassis_and_s100_integration.md) 对接 TODO 即可。

---

## 九、可观测性最小集（运行时）

部署后日常运行不需要 evo（GT 没有），但应该输出：

| Topic | 类型 | 用途 |
|-------|------|------|
| `/slam_toolbox/optimization_time` | `std_msgs/Float64`（自加 publisher）| 优化耗时监控 |
| `/diagnostics`（标准）| `diagnostic_msgs/DiagnosticArray` | 各节点状态 |
| `/tf_change_rate`（自定义）| `Float64` | map→odom 跳变频率 |
| Foxglove / PlotJuggler | — | 调试时可视 |

> 如何写自定义 publisher / 让 diagnostic_aggregator 工作 —— 见 `nav2_research/` 与 `rosbag2_research/`，本节不重复。

---

## 十、检查清单（在签字交付前打勾）

- [ ] 在 office_loop / corridor / lobby 三类数据集上跑过 5 次 baseline
- [ ] APE_RMSE_t / RPE_RMSE_t 中位数已记录到表格
- [ ] CPU 占用 < 60% 单核，内存 < 1GB
- [ ] 关闭 IMU / 错填协方差两种"故障注入"测过
- [ ] Lifelong 模式下连续运行 24h，vertex 数收敛、内存不爆
- [ ] 至少 1 次"参数误调"通过回归测试拦截
- [ ] 基线参数与基线指标一并提交到 git（参数 yaml）+ 大文件存储（数据集）

---

## 十一、与已有调研的关系

- `rosbag2_research/` — bag 录制 / 回放细节、MCAP 格式、压缩选型
- `simulation_research/` — Gazebo Harmonic 配置、`/clock` 同步
- `realtime_research/` — `ros2_tracing` 详解、CPU 隔离 / SCHED_FIFO 影响测量
- 本专题 [01](./01_slam_toolbox_deep_dive.md) §七 — 推荐起步参数即"baseline"

---

下一步：[07_chassis_and_s100_integration.md](./07_chassis_and_s100_integration.md) — 把这一切真正部署到 S100 主控上。
