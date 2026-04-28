# tf2 速查总结

## 核心 API 速查

| 操作 | 代码 |
|------|------|
| 创建 Buffer + Listener | `auto buf = make_shared<Buffer>(clock); auto lis = make_shared<TransformListener>(*buf);` |
| 查询变换（最新） | `buf->lookupTransform("target", "source", tf2::TimePointZero)` |
| 查询变换（精确时间） | `buf->lookupTransform("target", "source", stamp, timeout)` |
| 检查变换是否可用 | `buf->canTransform("target", "source", time)` |
| 发布动态变换 | `TransformBroadcaster br(node); br.sendTransform(tf_stamped)` |
| 发布静态变换 | `StaticTransformBroadcaster sbr(node); sbr.sendTransform(tf_stamped)` |
| 变换几何对象 | `tf2::doTransform(point_in, point_out, transform)` |

## /tf vs /tf_static 关键区别

| 特性 | /tf | /tf_static |
|------|-----|------------|
| QoS | BEST_EFFORT | RELIABLE + TRANSIENT_LOCAL |
| 历史深度 | 100 | 1 |
| 迟到订阅者 | 收不到历史 | 立即收到最新 |
| 适用场景 | 运动变换（里程计、关节） | 固定安装（传感器外参）|
| 发布频率 | 高频（10-100Hz） | 一次性 |

## 异常类型

| 异常 | 触发条件 |
|------|---------|
| `LookupException` | 帧名不存在 |
| `ConnectivityException` | source/target 不在同一坐标树 |
| `ExtrapolationException` | 请求时间超出缓存范围 |
| `InvalidArgumentException` | 四元数非单位向量、父子帧相同 |
| `TimeoutException` | waitForTransform 超时 |

## 坐标变换数学

```
T(A→C) = T(A→B) * T(B→C)

向量变换：  v_target = R * v_source                  （只旋转）
点变换：    p_target = R * p_source + t              （旋转+平移）
位姿变换：  pose_target = T_AB * pose_source         （完整变换）

取逆：T(B→A) = T(A→B)^-1
  translation: -R^T * t
  rotation:    R^T（转置）= q* （四元数共轭）
```

## 调试命令

```bash
# 查看当前 TF 树
ros2 run tf2_tools view_frames

# 实时查询变换
ros2 run tf2_ros tf2_echo odom base_link

# 检查 /tf topic
ros2 topic echo /tf

# 检查静态变换
ros2 topic echo /tf_static

# TF 树可视化（rqt）
rqt &  # → Plugins → Visualization → TF Tree

# 检查特定帧的父帧
ros2 run tf2_ros static_transform_publisher --print
```

## 常见问题排查

| 问题 | 可能原因 | 解决 |
|------|---------|------|
| ExtrapolationException | 请求时间比缓存最老还早（TF 发布延迟） | 增加 cache_time 或用 Time(0) |
| LookupException: frame does not exist | robot_state_publisher 未启动 | 检查 URDF 和 robot_state_publisher |
| ConnectivityException | 两个帧在不同树中（如未连接到 world） | 检查 TF 树完整性 |
| 变换落后（stale） | TF 发布频率太低 | 提高 robot_state_publisher 或控制器发布频率 |
| 静态变换丢失 | 新节点启动后未收到 /tf_static | 检查 StaticBroadcaster 的 QoS 是否为 TRANSIENT_LOCAL |
