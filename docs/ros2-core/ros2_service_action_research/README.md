# ROS2 Service / Action 深度调研 - 目录

## 文件列表

| 文件 | 内容 |
|------|------|
| [01_service_overview.md](./01_service_overview.md) | Service 架构、rcl/rmw 双 Topic 实现、请求-响应配对机制 |
| [02_service_source.md](./02_service_source.md) | rclcpp Service/Client 完整源码：create_service → DDS → 回调触发 |
| [03_action_overview.md](./03_action_overview.md) | Action 架构：5个 DDS Topic、状态机、GoalHandle 生命周期 |
| [04_action_source.md](./04_action_source.md) | rclcpp_action Server/Client 完整源码实现 |
| [05_summary.md](./05_summary.md) | 速查表、时序图、常见错误 |

## 核心原理

```
Service（请求-响应）：
  rclcpp::Service<T>  ←→  rcl_service_t  ←→  DDS Topic对：
    rq/<name>Request_  (DataWriter on client, DataReader on server)
    rr/<name>Reply_    (DataWriter on server, DataReader on client)

Action（目标-反馈-结果）：
  rclcpp_action::Server<T>  ←→  3个 rcl Service + 2个 rcl Publisher：
    rq/<name>/_action/send_goalRequest
    rr/<name>/_action/send_goalReply
    rq/<name>/_action/cancel_goalRequest
    rr/<name>/_action/cancel_goalReply
    rq/<name>/_action/get_resultRequest
    rr/<name>/_action/get_resultReply
    rt/<name>/_action/feedback      (Publisher)
    rt/<name>/_action/status        (Publisher)
```
