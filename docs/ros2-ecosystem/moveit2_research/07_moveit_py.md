# moveit_py — Python API 完整指南（取代 moveit_commander）

---

## 一、moveit_py 概述

`moveit_py` 是 MoveIt 2 的**官方 Python 绑定**（基于 pybind11），取代 ROS 1 时代的 `moveit_commander`：

| 特性 | moveit_commander（旧）| moveit_py（新）|
|------|:-----------------:|:----------:|
| ROS 版本 | ROS 1 + ROS 2 兼容层 | 原生 ROS 2 |
| 实现方式 | ROS 服务/话题封装 | 直接调用 C++ 库（pybind11）|
| 延迟 | 高（跨进程）| 低（进程内）|
| 规划场景访问 | 间接（通过 Service）| 直接（共享内存）|
| 维护状态 | 停止维护 | 活跃维护（MoveIt 2 官方）|

---

## 二、初始化与配置

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint

rclpy.init()

# MoveItPy 初始化（内部创建 move_group 连接）
moveit = MoveItPy(node_name="my_moveit_py_node")
# 获取规划组
arm = moveit.get_planning_component("arm")
gripper = moveit.get_planning_component("gripper")

# 获取机器人模型
robot_model = moveit.get_robot_model()
robot_state = RobotState(robot_model)
```

---

## 三、运动规划与执行

### 3.1 Pose 目标规划

```python
from geometry_msgs.msg import PoseStamped

def plan_to_pose(target_x, target_y, target_z, target_qw=1.0):
    """规划末端执行器到达指定 pose"""
    # 设置起始状态（当前关节状态）
    arm.set_start_state_to_current_state()

    # 构造目标 pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "world"
    goal_pose.pose.position.x = target_x
    goal_pose.pose.position.y = target_y
    goal_pose.pose.position.z = target_z
    goal_pose.pose.orientation.w = target_qw

    # 设置目标（指定末端执行器链接）
    arm.set_goal_state(
        pose_stamped_msg=goal_pose,
        pose_link="tool0"        # URDF 中末端执行器链接名
    )

    # 规划
    plan_result = arm.plan()
    if plan_result:
        robot_trajectory = plan_result.trajectory
        # 应用速度缩放
        robot_trajectory.apply_totg_time_parameterization(
            velocity_scaling_factor=0.5,
            acceleration_scaling_factor=0.5
        )
        # 执行（blocking=True 等待完成）
        moveit.execute(robot_trajectory, blocking=True, controllers=[])
        return True
    return False
```

### 3.2 关节目标规划

```python
def plan_to_joints(joint_values: dict):
    """规划到指定关节角度"""
    arm.set_start_state_to_current_state()

    # 构造关节约束
    robot_state = RobotState(moveit.get_robot_model())
    robot_state.joint_positions = joint_values
    # 例：{"joint1": 0.0, "joint2": -1.57, "joint3": 0.0, ...}

    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=moveit.get_robot_model()
                               .get_joint_model_group("arm")
    )
    arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    plan_result = arm.plan()
    if plan_result:
        moveit.execute(plan_result.trajectory, blocking=True)
        return True
    return False

# 预设命名位置（在 SRDF 中定义）
def plan_to_named_pose(name: str):
    """规划到 SRDF 中定义的命名位置（如 'home', 'ready'）"""
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    plan_result = arm.plan()
    if plan_result:
        moveit.execute(plan_result.trajectory, blocking=True)
```

### 3.3 规划器选择

```python
# 切换规划器（OMPL / Pilz / STOMP）
from moveit.planning import PlanRequestParameters

plan_params = PlanRequestParameters(moveit, "arm")
plan_params.planner_id = "LIN"            # Pilz 直线运动
plan_params.planning_pipeline = "pilz_industrial_motion_planner"
plan_params.planning_time = 1.0           # s，规划超时
plan_params.planning_attempts = 5         # 最多尝试次数
plan_params.max_velocity_scaling_factor = 0.5
plan_params.max_acceleration_scaling_factor = 0.5

plan_result = arm.plan(plan_params)
```

---

## 四、Planning Scene 管理

### 4.1 添加 / 删除碰撞对象

```python
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def add_collision_box(name, size, position):
    """在 planning scene 中添加矩形障碍物"""
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = "world"

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size)         # [x, y, z] 尺寸
    obj.primitives.append(box)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.w = 1.0
    obj.primitive_poses.append(pose)

    obj.operation = CollisionObject.ADD

    # 通过 planning_scene_monitor 更新（线程安全）
    with moveit.get_planning_scene_monitor().read_write() as scene:
        scene.apply_collision_object(obj)
        scene.current_state.update()

def remove_collision_object(name):
    obj = CollisionObject()
    obj.id = name
    obj.operation = CollisionObject.REMOVE
    with moveit.get_planning_scene_monitor().read_write() as scene:
        scene.apply_collision_object(obj)

def clear_all_collision_objects():
    """清空所有碰撞对象"""
    with moveit.get_planning_scene_monitor().read_write() as scene:
        scene.clear()
```

### 4.2 抓取与放置（Attached Object）

```python
def attach_object(object_id, link_name, touch_links=None):
    """机器人抓取物体后附着到指定链接"""
    with moveit.get_planning_scene_monitor().read_write() as scene:
        scene.current_state.attach_body(
            object_id=object_id,
            link_name=link_name,
            touch_links=touch_links or []
        )

def detach_object(object_id):
    """放置物体后解除附着"""
    with moveit.get_planning_scene_monitor().read_write() as scene:
        scene.current_state.detach_body(object_id)

# 典型抓取流程
def pick_and_place(pick_pose, place_pose, obj_id):
    # 1. 移动到预抓取位置
    plan_to_pose(*pick_pose_pre)
    # 2. 夹爪张开
    gripper.set_goal_state(configuration_name="open")
    moveit.execute(gripper.plan().trajectory, blocking=True)
    # 3. 直线下降
    plan_to_pose(*pick_pose)
    # 4. 夹爪闭合
    gripper.set_goal_state(configuration_name="close")
    moveit.execute(gripper.plan().trajectory, blocking=True)
    # 5. 附着物体（碰撞检测包含物体）
    attach_object(obj_id, "tool0", touch_links=["finger_left","finger_right"])
    # 6. 上升到安全高度
    plan_to_pose(*pick_pose_lift)
    # 7. 移动到放置位置
    plan_to_pose(*place_pose)
    # 8. 夹爪张开 + 解除附着
    gripper.set_goal_state(configuration_name="open")
    moveit.execute(gripper.plan().trajectory, blocking=True)
    detach_object(obj_id)
```

---

## 五、机器人状态查询

```python
# 获取当前关节状态
with moveit.get_planning_scene_monitor().read_only() as scene:
    current_state = scene.current_state
    joint_positions = current_state.get_joint_group_positions("arm")
    # → numpy array，按 joint_model_group 中关节顺序排列

# 获取末端执行器当前位姿（FK 正运动学）
    ee_pose = current_state.get_global_link_transform("tool0")
    # → 4x4 numpy 变换矩阵

# 碰撞检测
    is_valid = scene.is_state_valid(current_state, "arm")
```

---

## 六、约束规划（Orientation / Position）

```python
from moveit.core.kinematic_constraints import (
    construct_orientation_constraint,
    construct_position_constraint
)
from geometry_msgs.msg import QuaternionStamped, PointStamped

# 方向约束（保持末端执行器朝下）
orient_const = construct_orientation_constraint(
    link_name="tool0",
    quaternion_stamped=QuaternionStamped(
        header=...,
        quaternion=Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)  # 朝下
    ),
    absolute_x_axis_tolerance=0.4,  # rad
    absolute_y_axis_tolerance=0.4,
    absolute_z_axis_tolerance=0.4,
    weight=1.0
)

arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="tool0")
arm.set_path_constraints(moveit_msgs.msg.Constraints(
    orientation_constraints=[orient_const]
))
plan_result = arm.plan()
```
