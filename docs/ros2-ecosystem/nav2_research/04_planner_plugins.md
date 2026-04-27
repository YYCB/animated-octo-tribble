# 全局规划器插件对比 — NavFn / SmacPlanner / ThetaStar

---

## 一、全局规划器概览

全局规划器（Global Planner）负责在**全局代价地图**上计算从起点到目标点的**全局路径**，供 controller_server 跟踪。

接口：`nav2_core::GlobalPlanner`

```cpp
// 所有全局规划器实现的接口
class GlobalPlanner {
  virtual void configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
                         std::string name,
                         std::shared_ptr<tf2_ros::Buffer> tf,
                         std::shared_ptr<Costmap2DROS> costmap_ros) = 0;
  virtual nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) = 0;
};
```

---

## 二、NavFn（Dijkstra / A*）

### 2.1 算法

基于**网格扩散**的全局规划器，是 Nav2 最早移植的规划器（来自 ROS 1 `navfn` 包）。

```
算法：Dijkstra（use_astar: false）或 A*（use_astar: true）
搜索空间：Costmap2D 栅格（grid cell 级别）
输出路径：通过梯度下降在势场上平滑的路径点序列
```

### 2.2 特点与局限

| 优点 | 缺点 |
|------|------|
| 实现简单成熟，稳定性高 | 路径质量差（折线多，不平滑）|
| 内存占用低 | 不支持朝向（Holonomic-only）|
| 规划速度快（小地图）| 对狭窄走廊容易规划到障碍边缘 |
| 适合简单室内场景 | 大地图时速度慢（O(n²) 扩展）|

### 2.3 配置

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5         # m，目标点容忍半径（目标附近有障碍时用）
      use_astar: true        # false=Dijkstra，true=A*
      allow_unknown: true    # 是否允许路径经过未知区域
```

**适用场景**：简单室内环境，快速原型，差速驱动机器人（无方向约束）。

---

## 三、SmacPlanner（推荐，主力规划器）

SmacPlanner 是 Nav2 的**新一代规划器**，提供三种变体，解决了 NavFn 的主要缺陷。

### 3.1 SmacPlanner2D — 改进型网格 A*

```
算法：在 Costmap 网格上运行的 A*，带平滑后处理
特点：
  - 8-连通（NavFn 的梯度下降 → 更准确的路径）
  - 支持代价感知（高代价区域绕行）
  - 内置路径平滑（Ceres 优化）
  - 不支持方向约束（非完整约束）
适用：差速驱动，要求路径质量好于 NavFn
```

```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.125               # m，目标容忍
      downsample_costmap: false       # 是否降采样（大地图时加速）
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0          # s，规划超时
      cost_travel_multiplier: 2.0     # 代价权重（越高越倾向走低代价路）
      use_final_approach_orientation: true
      smoother:
        max_iterations: 1000
        w_smooth: 0.3                 # 平滑权重
        w_data: 0.2                   # 贴合原始路径权重
        tolerance: 1.0e-10
        do_refinement: true
```

### 3.2 SmacPlannerHybrid — Hybrid-A*（适合阿克曼/前驱）

```
算法：Hybrid-A*（Dolgov et al.，Reeds-Shepp/Dubin 曲线扩展）
特点：
  - 考虑机器人朝向（SE2 搜索空间：x, y, θ）
  - 满足最小转弯半径约束（阿克曼转向，最重要！）
  - 支持倒车（reverse: true）
  - Ceres 路径平滑（曲率连续）
适用：阿克曼底盘（无人车、叉车）、需要朝向约束的场景
```

```yaml
HybridPlanner:
  plugin: "nav2_smac_planner/SmacPlannerHybrid"
  downsample_costmap: false
  downsampling_factor: 1
  tolerance: 0.25
  allow_unknown: true
  max_iterations: 1000000
  max_planning_time: 5.0
  motion_model_for_search: "REEDS_SHEPP"  # 或 "DUBIN"（单向，更快）
  angle_quantization_bins: 72             # θ 离散化精度（360/72=5°）
  analytic_expansion_ratio: 3.5           # 解析扩展频率（提高规划速度）
  analytic_expansion_max_length: 3.0
  minimum_turning_radius: 0.40            # m，最小转弯半径（阿克曼约束）
  reverse_penalty: 2.1                    # 倒车代价惩罚（越高越不喜欢倒车）
  change_penalty: 0.0
  non_straight_penalty: 1.20             # 转弯代价惩罚
  cost_penalty: 2.0
  retrospective_penalty: 0.025
  lookup_table_size: 20.0                 # m，扩展查找表尺寸
  cache_obstacle_heuristic: false
  smoother:
    max_iterations: 1000
    w_smooth: 0.3
    w_data: 0.2
    tolerance: 1.0e-10
    do_refinement: true
    refinement_num: 2
```

### 3.3 SmacPlannerLattice — State Lattice（任意复杂运动学）

```
算法：State Lattice（预计算运动图元，最通用）
特点：
  - 通过外部工具生成运动图元（primitives），可以表达任意运动学约束
  - 支持差速/阿克曼/全向/履带 等任意底盘
  - 路径质量最高（理论上）
  - 需要预生成 lattice primitives 文件（相对复杂）
适用：有复杂运动学约束的底盘，追求最优路径质量
```

---

## 四、ThetaStar Planner — 任意角度 A*

```
算法：Theta*（Any-Angle A*）
特点：
  - 突破 A* 的网格对齐限制，可以规划任意角度的直线路径
  - 路径更直（不像 NavFn/SmacPlanner2D 那样沿网格锯齿）
  - 不考虑机器人朝向（仍是 Holonomic）
适用：要求路径较直、无方向约束的差速机器人
```

```yaml
ThetaStar:
  plugin: "nav2_theta_star_planner/ThetaStarPlanner"
  how_many_corners: 8           # 4=四连通，8=八连通
  w_euc_cost: 1.0               # 欧氏距离权重
  w_traversal_cost: 2.0         # 代价地图遍历权重
  w_heuristic_param: 1.0        # 启发式权重（>1 更快但不一定最优）
```

---

## 五、规划器横向对比

| 规划器 | 方向约束 | 路径质量 | 规划速度 | 适合底盘 | 复杂度 |
|--------|:------:|:------:|:------:|---------|:----:|
| NavFn（Dijkstra）| ❌ | ⭐⭐ | ⭐⭐⭐⭐⭐ | 差速/全向 | 低 |
| NavFn（A*）| ❌ | ⭐⭐ | ⭐⭐⭐⭐ | 差速/全向 | 低 |
| SmacPlanner2D | ❌ | ⭐⭐⭐ | ⭐⭐⭐⭐ | 差速/全向 | 中 |
| SmacPlannerHybrid | ✅（最小转弯半径）| ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | **阿克曼**（无人车）| 中 |
| SmacPlannerLattice | ✅（任意运动学）| ⭐⭐⭐⭐⭐ | ⭐⭐ | 任意（需 primitives）| 高 |
| ThetaStar | ❌ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 差速/全向 | 低 |

---

## 六、选型建议（本仓库 chassis_protocol）

`chassis_protocol` 目前实现的是**差速驱动底盘**（`DiffDriveController`），因此：

```
推荐：SmacPlanner2D（路径质量好，无朝向约束）
备选：ThetaStar（路径更直）
不推荐：SmacPlannerHybrid（差速底盘不需要最小转弯半径约束）

关键参数（差速底盘）：
  minimum_turning_radius: 0.0     # 差速底盘可原地旋转，无最小转弯半径
  motion_model: "VON_NEUMANN"     # 或 "MOORE"（8连通）
```
