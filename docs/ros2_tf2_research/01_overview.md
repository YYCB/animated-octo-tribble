# tf2（一）：架构与数学模型

## 一、tf2 的定位与历史

```
tf (ROS1)  → 全局单一坐标树，单节点中心化，有单点故障风险
           → 基于 Bullet 物理引擎数学库
           
tf2 (ROS2) → 分布式：每个节点都维护完整坐标树副本
           → 纯 C++ tf2 核心库（无 ROS 依赖）
           → 通过 /tf + /tf_static Topic 广播变换
           → tf2_ros：ROS2 集成层（Buffer, Listener, Broadcaster）
           → tf2_geometry_msgs：与 geometry_msgs 互转
           → tf2_eigen：与 Eigen 互转
           → tf2_bullet：与 Bullet 互转
```

---

## 二、坐标树数据模型

```
坐标帧（Frame）：命名的三维坐标系，例如：
  world / odom / base_link / base_laser / camera_link / camera_optical

坐标树（Transform Tree）：各帧之间的父子关系
  world
  └── odom
      └── base_link
          ├── base_laser
          ├── imu_link
          └── camera_link
              └── camera_optical_frame

边（Transform）：每条父→子边 = 平移 + 旋转（四元数）
  T(parent→child) = {
    header.frame_id = "base_link"   // 父帧
    child_frame_id  = "base_laser"  // 子帧
    transform.translation = {x, y, z}
    transform.rotation    = {x, y, z, w}  // 单位四元数
    header.stamp = rclcpp::Time      // 时间戳（动态变换用）
  }

静态变换 vs 动态变换：
  - 静态（/tf_static）：物理固定的部件（传感器外参），发布一次永久有效
    QoS：RELIABLE + TRANSIENT_LOCAL（新节点加入可收到历史消息）
  - 动态（/tf）：运动变化的部件（机器人位姿、关节角），需持续发布
    QoS：BEST_EFFORT + VOLATILE（历史消息无需持久化）
```

---

## 三、变换链查找的数学原理

```
目标：求 source_frame 到 target_frame 的变换

例：查询 camera_optical 到 odom 的变换：

  odom → base_link → camera_link → camera_optical

算法（walkToTopParent）：
  1. 从 source（camera_optical）向上走到根（world）
     路径1：camera_optical → camera_link → base_link → odom
     
  2. 从 target（odom）向上走到根
     路径2：odom → world

  3. 找到 LCA（最近公共祖先）= odom（在此例中 target 本身就是祖先）

  4. 合并变换：
     T(source→target) = T(odom→base_link)^-1 
                       * T(base_link→camera_link)^-1 
                       * T(camera_link→camera_optical)^-1

     注：从 child 到 parent = 取逆变换

变换合并（乘法）：
  T_AB * T_BC = T_AC
  
  translation: t_AC = R_AB * t_BC + t_AB
  rotation:    q_AC = q_AB * q_BC  (四元数乘法)
```

---

## 四、时间插值机制

```
动态变换的时间管理：
  每条边有一个 TimeCache（时间序列缓冲区）
  默认缓存 10 秒的历史变换（cache_time 参数）
  
  请求时间 t 的变换：
    若 t 在缓存范围内：SLERP 球面线性插值
    若 t 早于最早记录：ExtrapolationException
    若 t 晚于最新记录且超过 extrapolation_limit：ExtrapolationException

SLERP 四元数插值：
  q(t) = q0 * (q0^-1 * q1)^alpha
  alpha = (t - t0) / (t1 - t0)
  
线性平移插值：
  p(t) = p0 + alpha * (p1 - p0)

特殊时间 t = Time(0)：
  → 使用最新时间的变换（最近可用数据）
  → 常用于只需要当前位姿而不关心精确时间的场景
```

---

## 五、tf2 包结构

```
tf2/                    ← 纯 C++ 核心库（无 ROS 依赖）
  ├── buffer_core.h/.cpp    ← 核心变换树
  ├── time_cache.h/.cpp     ← 时间序列缓存
  ├── exceptions.h          ← 自定义异常
  ├── transform_datatypes.h ← Stamped<T>, StampedTransform
  └── LinearMath/           ← 向量、四元数、变换矩阵（来自 Bullet）

tf2_ros/               ← ROS2 集成层
  ├── buffer.h/.cpp         ← Buffer：继承 BufferCore + 异步 API
  ├── transform_listener.h/.cpp  ← 订阅 /tf + /tf_static
  ├── transform_broadcaster.h/.cpp ← 发布到 /tf
  ├── static_transform_broadcaster.h/.cpp ← 发布到 /tf_static
  ├── message_filter.h      ← 等待 tf 可用后触发消息处理
  └── create_timer.h        ← tf2::Duration 到 rclcpp::Duration 转换

tf2_geometry_msgs/     ← geometry_msgs 类型支持
  └── tf2_geometry_msgs.h   ← doTransform(), fromMsg(), toMsg()
```

---

## 六、关键数据类型

```cpp
// geometry_msgs/msg/TransformStamped.msg
// → 这是 tf2 的基础消息类型

Header header          // frame_id = 父帧, stamp = 时间戳
string child_frame_id  // 子帧
Transform transform
  Vector3 translation
    float64 x, y, z
  Quaternion rotation
    float64 x, y, z, w

// tf2::Transform（内部表示，基于 Bullet LinearMath）
class Transform {
  Quaternion m_basis;    // 旋转（也可以用 Matrix3x3）
  Vector3    m_origin;   // 平移
  
  Transform operator*(const Transform& t) const {
    return Transform(m_basis * t.m_basis,
                     (*this)(t.m_origin));
  }
  
  Transform inverse() const {
    Quaternion inv = m_basis.inverse();
    return Transform(inv, inv * (-m_origin));
  }
};

// tf2::Stamped<T>：带时间戳的数据
template<typename T>
struct Stamped : public T {
  tf2::TimePoint stamp_;
  std::string frame_id_;
};
```

---

## 七、BufferCore 内部存储结构

```cpp
// tf2/src/buffer_core.cpp

class BufferCore {
  // ★ 帧名到帧ID的映射（字符串 → 整数，节省内存）
  typedef uint32_t CompactFrameID;
  
  std::vector<TimeCacheInterfacePtr> frames_;   // frames_[frameID] = 时间缓存
  std::unordered_map<std::string, CompactFrameID> frameIDs_;  // 名字→ID
  std::vector<std::string> frameIDs_reverse_;  // ID→名字（逆映射）
  
  mutable std::mutex frame_mutex_;    // 并发保护（多线程安全）
};

// 每条边的时间缓存：
class TimeCache : public TimeCacheInterface {
  // 有序双端队列，按时间排序
  typedef std::deque<TransformStorage> L_TransformStorage;
  L_TransformStorage storage_;   // 最多 cache_time 秒的历史数据
  
  // TransformStorage：最小化内存占用
  struct TransformStorage {
    tf2::TimePoint stamp_;
    Quaternion rotation_;         // 4 × float64 = 32字节
    Vector3    translation_;      // 3 × float64 = 24字节
    CompactFrameID frame_id_;     // uint32 = 4字节
    CompactFrameID child_frame_id_;
    uint32_t    tid_;             // 线程 ID（调试用）
    // 总计：~68字节/条
  };
};
```

---

## 八、TF 坐标变换的完整数学示例

```
场景：机器人底盘（base_link）在世界坐标系（world）中
      激光雷达挂载在底盘上，偏移 [0.1, 0, 0.3]

已知：
  T(world→base_link) = { trans=[5,3,0], rot=[0,0,sin(π/4),cos(π/4)] }
                        （机器人在 world 位置 (5,3,0)，偏航 45°）
  T(base_link→base_laser) = { trans=[0.1,0,0.3], rot=[0,0,0,1] }
                              （静态，无旋转）

查询：激光点 p_laser=[1,0,0] 在 world 中的位置？

Step 1: 变换到 base_link
  T(world→base_link) * p_laser
  → p_base = R_wb * p_laser + t_wb
  → p_base = rot45° * [1,0,0] + [0.1,0,0.3]
           = [cos45°, sin45°, 0] + [0.1,0,0.3]
           = [0.807, 0.707, 0.3]

Step 2: 变换到 world
  p_world = R_bw * p_base + t_bw
  ...

tf2 自动完成以上计算：
  auto t = tf_buffer.lookupTransform("world", "base_laser", tf2::TimePointZero);
  tf2::doTransform(point_in_laser, point_in_world, t);
```
