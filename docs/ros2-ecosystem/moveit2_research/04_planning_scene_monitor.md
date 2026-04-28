# PlanningSceneMonitor — 实时碰撞场景维护与性能优化

---

## 一、PlanningSceneMonitor 的角色

`planning_scene_monitor::PlanningSceneMonitor` 是 MoveIt 2 中**维护全局 planning scene 状态**的核心组件，由 move_group 内部创建并持续更新：

```
PlanningSceneMonitor 维护的信息：
  1. 机器人当前构型（joint state）
  2. 机器人当前世界位姿（TF：world/odom → base_link）
  3. 碰撞场景对象（geometry primitives / mesh / Octomap）
  4. 附着对象（attached collision objects，如抓取的物体）
  5. 约束（workspace bounds）

更新来源：
  /joint_states          → CurrentStateMonitor（关节角）
  /tf                    → TF 监听器（机器人位姿）
  /planning_scene        → PlanningScene 差量消息（外部添加对象）
  /collision_object      → CollisionObject 消息（添加/移除场景物体）
  /attached_collision_object → AttachedCollisionObject
  深度传感器            → OccupancyMapMonitor（Octomap）
```

---

## 二、核心数据结构

```cpp
// planning_scene::PlanningScene
// 线程安全访问方式：
planning_scene_monitor::LockedPlanningSceneRW scene(psm);  // 读写锁
// 或
planning_scene_monitor::LockedPlanningSceneRO scene(psm);  // 只读锁

// 检查碰撞
collision_detection::CollisionRequest req;
collision_detection::CollisionResult res;
scene->checkCollision(req, res);  // 检查当前构型

// 添加碰撞对象
moveit_msgs::msg::CollisionObject obj;
obj.id = "box1";
obj.header.frame_id = "world";
obj.primitives.push_back(make_box(0.1, 0.1, 0.1));
obj.primitive_poses.push_back(make_pose(1.0, 0.0, 0.5));
obj.operation = moveit_msgs::msg::CollisionObject::ADD;
scene->processCollisionObjectMsg(obj);
```

---

## 三、性能瓶颈分析

### 3.1 主要性能热点

| 操作 | 典型耗时 | 触发频率 | 影响 |
|------|---------|---------|------|
| 碰撞检测（FCL，6轴简单mesh）| 0.1-1ms | 每次规划采样（1000+次/规划）| 规划时间主要来源 |
| Octomap 更新（点云积分）| 5-50ms | 10-30 Hz（传感器频率）| 规划场景更新延迟 |
| PlanningScene 锁竞争 | 0.01-5ms | 规划 + 传感器并发 | 读写锁争用 |
| TF 查询（per 碰撞检测）| 0.01-0.1ms | 每次碰撞检测 | 累积影响 |
| planning_scene 发布 | 1-10ms | 每次场景变化 | 网络/序列化开销 |

### 3.2 锁争用问题

```
PlanningSceneMonitor 使用单一读写锁（boost::shared_mutex）：

规划线程：需要 ReadLock（频繁）
传感器更新线程：需要 WriteLock（频繁）
→ 大量传感器更新 + 高频规划 = 严重锁争用

症状：
  - 规划时间突然变长（10x 以上）
  - /planning_scene 话题延迟大
  - CPU 不高但规划很慢
```

---

## 四、性能优化策略

### 4.1 碰撞检测优化

```yaml
# SRDF 优化：禁用不可能碰撞的链接对
# 使用 MoveIt Setup Assistant 生成的碰撞矩阵
<disable_collisions link1="base_link" link2="upper_arm_link" reason="Never"/>

# 碰撞 mesh 简化：在 URDF 中使用凸包（convex hull）代替精确 mesh
# 工具：V-HACD（Volumetric Hierarchical Approximate Convex Decomposition）
```

```python
# 动态调整碰撞检测粒度（规划时）
req = MotionPlanRequest()
req.allowed_planning_time = 5.0
# 减少碰撞检测频率：增大 longest_valid_segment_fraction（降低精度换速度）
# 在 ompl_planning.yaml 中：
# longest_valid_segment_fraction: 0.01  → 0.05（精度降低5倍，速度提升）
```

### 4.2 Octomap 优化

```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    max_range: 3.0              # ← 减小点云使用范围（2.0~3.0 即可）
    point_subsample: 3          # ← 每3个点取1个（降采样，减少 Octomap 更新负载）
    octomap_resolution: 0.05    # ← 增大分辨率（0.05 比 0.02 快 ~8x）
    filtered_cloud_topic: /filtered_cloud
```

```yaml
# move_group 参数：
octomap_resolution: 0.05        # m，不要低于 0.03（除非必要）
max_update_rate: 5.0            # Hz，限制 Octomap 更新频率（默认无限制）
```

### 4.3 PlanningScene 更新策略

```cpp
// 批量更新（减少锁争用）：
{
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    // 在同一个锁内完成所有修改
    scene->processCollisionObjectMsg(obj1);
    scene->processCollisionObjectMsg(obj2);
    scene->processAttachedCollisionObjectMsg(attached_obj);
}  // 锁释放，触发一次发布

// 对比：逐个更新（每次都要重新加锁，效率低）
```

### 4.4 规划期间冻结传感器更新

```cpp
// 规划前暂停传感器更新（避免规划中途场景变化）
psm_->lockSceneRead();   // 获取读锁（阻止传感器写入）
auto req = MotionPlanRequest{...};
auto result = planner_->solve(req);
psm_->unlockSceneRead();  // 释放
```

### 4.5 使用 Octree Lazy Evaluation

```yaml
# 延迟 Octomap 更新（只在需要规划时才完整更新）
octomap_monitor:
  update_interval: 0.2     # s，每 200ms 更新一次 Octomap（而非每帧）
```

---

## 五、场景对象管理 API

### 5.1 通过 MoveGroupInterface（Python）

```python
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

moveit = MoveItPy(node_name="moveit_py_node")
planning_scene_interface = moveit.get_planning_scene_monitor()

# 添加盒子
def add_box_to_scene(name, size, pose_xyz):
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = "world"
    
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size)
    obj.primitives.append(box)
    
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = pose_xyz
    pose.orientation.w = 1.0
    obj.primitive_poses.append(pose)
    
    obj.operation = CollisionObject.ADD
    
    with planning_scene_interface.read_write() as scene:
        scene.apply_collision_object(obj)
        scene.current_state.update()  # 强制更新

add_box_to_scene("table", (1.0, 0.8, 0.1), (0.5, 0.0, 0.4))
```

### 5.2 抓取后附着对象（Attached Collision Object）

```python
# 机器人抓取物体后，将物体附着到末端执行器
# （附着后物体随机器人移动，参与碰撞检测）
with planning_scene_interface.read_write() as scene:
    scene.attach_object(
        object_id="target_object",
        link_name="gripper_link",
        touch_links=["finger_left", "finger_right"]  # 允许与手指碰撞
    )

# 放置后解除附着
with planning_scene_interface.read_write() as scene:
    scene.detach_object("target_object")
```

---

## 六、监控性能的工具

```bash
# 查看 planning_scene 话题延迟
ros2 topic delay /planning_scene

# 查看 Octomap 更新频率
ros2 topic hz /move_group/monitored_planning_scene

# 可视化（RViz）
# Add → MotionPlanning → Planning Scene → 查看碰撞对象
# Add → PointCloud2 → /filtered_cloud → 查看 Octomap 过滤结果
```
