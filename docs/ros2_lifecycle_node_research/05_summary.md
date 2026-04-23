# Lifecycle Node 速查总结

## 状态与转换速查

| 转换名 | from → to | Transition ID |
|--------|-----------|---------------|
| configure | UNCONFIGURED → INACTIVE | 1 |
| cleanup | INACTIVE → UNCONFIGURED | 2 |
| activate | INACTIVE → ACTIVE | 3 |
| deactivate | ACTIVE → INACTIVE | 4 |
| shutdown | ANY → FINALIZED | 5/6/7 |
| error | ERRORPROCESSING → FINALIZED | 9 |

## 5 个回调速查

| 回调 | 转换 | 任务 |
|------|------|------|
| `on_configure()` | UNCONFIGURED→INACTIVE | 读参数、创建 Publisher/Subscriber、初始化硬件 |
| `on_activate()` | INACTIVE→ACTIVE | 激活 LifecyclePublisher、启动 Timer |
| `on_deactivate()` | ACTIVE→INACTIVE | 取消 Timer、去激活 LifecyclePublisher |
| `on_cleanup()` | INACTIVE→UNCONFIGURED | 释放所有资源（reset unique_ptr） |
| `on_shutdown()` | ANY→FINALIZED | 紧急清理（快速退出） |

## CallbackReturn 含义

| 返回值 | 效果 |
|--------|------|
| `SUCCESS` | 转换成功，进入目标 Primary State |
| `FAILURE` | 转换失败，回到 previous Primary State |
| `ERROR` | 严重错误，进入 ERRORPROCESSING |

## 调试命令

```bash
# 查看节点当前状态
ros2 lifecycle get /my_lifecycle_node

# 触发状态转换
ros2 lifecycle set /my_lifecycle_node configure
ros2 lifecycle set /my_lifecycle_node activate
ros2 lifecycle set /my_lifecycle_node deactivate
ros2 lifecycle set /my_lifecycle_node cleanup
ros2 lifecycle set /my_lifecycle_node shutdown

# 列出所有 Lifecycle 节点
ros2 lifecycle list

# 监听状态变化
ros2 topic echo /my_lifecycle_node/transition_event
```

## 资源管理最佳实践

```
构造函数：只做参数声明，不分配资源
on_configure：分配资源（硬件、Publisher）
on_activate：开始工作（Timer、LifecyclePublisher.on_activate()）
on_deactivate：停止工作（cancel Timer、.on_deactivate()）
on_cleanup：释放资源（reset()）
on_shutdown：快速释放（同 cleanup，但可以不完整）
```
