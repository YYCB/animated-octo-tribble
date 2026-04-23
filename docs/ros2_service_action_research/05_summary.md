# Service/Action 速查总结

## Service 速查

| 场景 | 代码 |
|------|------|
| 创建 Server | `node->create_service<T>(name, callback)` |
| 创建 Client | `node->create_client<T>(name)` |
| 等待 Server | `client->wait_for_service(timeout)` |
| 发送请求 | `client->async_send_request(request)` |
| 同步等待 | `executor.spin_until_future_complete(future, timeout)` |

## Action 速查

| 场景 | 代码 |
|------|------|
| 创建 Server | `rclcpp_action::create_server<T>(node, name, goal_cb, cancel_cb, accepted_cb)` |
| 创建 Client | `rclcpp_action::create_client<T>(node, name)` |
| 发送目标 | `client->async_send_goal(goal, options)` |
| 发布 Feedback | `goal_handle->publish_feedback(fb)` |
| 报告成功 | `goal_handle->succeed(result)` |
| 检查取消 | `goal_handle->is_canceling()` |
| 取消目标 | `client->async_cancel_goal(goal_handle)` |

## Service vs Topic vs Action 对比

| 维度 | Topic | Service | Action |
|------|-------|---------|--------|
| 通信模式 | 发布-订阅 | 请求-响应 | 目标-反馈-结果 |
| DDS Topic数 | 1 | 2 | 8 |
| QoS | 用户配置 | 固定 RELIABLE | 固定 RELIABLE + 自定义 |
| 阻塞 | 否 | 短时间 | 长时间（可取消）|
| 反馈 | 无 | 无 | 实时 Feedback |
| 适用场景 | 传感器数据 | 计算/查询 | 导航/抓取/长任务 |

## 常见错误

| 错误 | 原因 | 解决 |
|------|------|------|
| `wait_for_service` 超时 | 服务端未启动 | 先启动 Server 节点 |
| `async_send_request` 无响应 | Executor 未 spin | 保证 spin 在另一线程/节点 |
| `is_canceling()` 未检查 | 执行逻辑不响应取消 | 在循环中定期检查 |
| Goal 状态卡在 EXECUTING | 未调用 succeed/abort | Server 执行函数必须在终态调用 succeed/abort/canceled |
| `get_result` 挂起 | Server 进程崩溃 | 增加超时处理 |

## DDS Topic 命名规则

```
/my_service
  → rq/my_service:MySrv_Request_
  → rr/my_service:MySrv_Reply_

/my_action
  → rq/my_action/_action/send_goalRequest_
  → rr/my_action/_action/send_goalReply_
  → rq/my_action/_action/cancel_goalRequest_
  → rr/my_action/_action/cancel_goalReply_
  → rq/my_action/_action/get_resultRequest_
  → rr/my_action/_action/get_resultReply_
  → rt/my_action/_action/feedback
  → rt/my_action/_action/status
```

## 调试命令

```bash
# 列出所有 Service
ros2 service list

# 调用 Service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# 列出所有 Action
ros2 action list

# 发送 Action 目标
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# 带反馈
ros2 action send_goal -f /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# 监控 Service 消息（Humble+）
ros2 service echo /add_two_ints
```
