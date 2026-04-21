# 08 关键数据流时序图

## 1. 连接建立流程

```
ChassisNode          ChassisHal           VersionNegotiator    ITransport       MCU 固件
    │                    │                       │                 │               │
    │ initController()   │                       │                 │               │
    │──────────────────►│                       │                 │               │
    │                    │ connect()             │                 │               │
    │                    │──────────────────────────────────────►│               │
    │                    │                       │   connectSocket/open()         │
    │                    │                       │                 │──────────────►│ (TCP SYN / RS485 打开)
    │                    │                       │                 │◄──────────────│ (ACK)
    │                    │◄──────────────────────────────────────│ transport::connect() OK
    │                    │                       │                 │               │
    │                    │ setReceiveCallback()  │                 │               │
    │                    │──────────────────────────────────────►│               │
    │                    │                       │                 │               │
    │                    │ negotiator_.reset()   │                 │               │
    │                    │──────────────────────►│                 │               │
    │                    │                       │                 │               │
    │                    │ negotiator_.sendHandshake()             │               │
    │                    │──────────────────────►│                 │               │
    │                    │                       │ buildHandshakeReqPayload()      │
    │                    │                       │ encode(HANDSHAKE_REQ)           │
    │                    │                       │ transport_.send()               │
    │                    │                       │────────────────►│               │
    │                    │                       │                 │ write()────►  │
    │                    │                       │                 │               │
    │                    │   [等待 0~2000ms]      │                 │               │
    │                    │                       │                 │    HANDSHAKE_RESP 帧
    │                    │                       │                 │◄──────────────│
    │                    │                       │                 │               │
    │                    │ onDataReceived()       │ receiveLoop     │               │
    │                    │◄───────────────────────────────────────│               │
    │                    │ stream_decoder_.feed() │                 │               │
    │                    │ onFrameDecoded()       │                 │               │
    │                    │ negotiator_.onFrameReceived(HANDSHAKE_RESP)             │
    │                    │──────────────────────►│                 │               │
    │                    │                       │ handleHandshakeResp()           │
    │                    │                       │ state_ = CONNECTED              │
    │                    │                       │                 │               │
    │                    │ connected_ = true      │                 │               │
    │                    │ 启动 reconnectLoop 线程 │                 │               │
    │◄──────────────────│ connect() 返回 true    │                 │               │
    │                    │                       │                 │               │
```

---

## 2. 下行命令流程（BODY_VELOCITY 模式）

```
ROS2 spin线程              ChassisHal              ITransport           MCU 固件
      │                        │                        │                  │
      │ /cmd_vel 回调          │                        │                  │
      │ sendVelocity(vx,vy,ω) │                        │                  │
      │───────────────────────►│                        │                  │
      │                        │ buildVelocityPayload() │                  │
      │                        │ [seq, vx, vy, ω, ts]   │                  │
      │                        │ FrameCodec::encode(VELOCITY_CMD)          │
      │                        │ [AA 55 01 10 LEN CRC]  │                  │
      │                        │ transport_->send()     │                  │
      │                        │───────────────────────►│                  │
      │                        │                        │ write(fd, frame) │
      │                        │                        │─────────────────►│
      │                        │                        │                  │ 解析帧
      │                        │                        │                  │ 执行逆运动学
      │                        │                        │                  │ 输出各轮 PWM
      │◄──────────────────────│ send() 返回 true/false  │                  │
```

---

## 3. 下行命令流程（WHEEL_RPM 模式）

```
ROS2 spin线程       DifferentialController   ChassisHal    ITransport      MCU
      │                     │                    │              │            │
      │ /cmd_vel 回调        │                    │              │            │
      │ sendVelocity(vx,0,ω)│                    │              │            │
      │────────────────────►│                    │              │            │
      │                     │ clamp(vx, ω)       │              │            │
      │                     │ IK: w_L = (vx-L/2*ω)/r           │            │
      │                     │     w_R = (vx+L/2*ω)/r           │            │
      │                     │ ChassisHal::sendWheelCmd([w_L,w_R])           │
      │                     │───────────────────►│              │            │
      │                     │                    │ buildWheelPayload()       │
      │                     │                    │ [seq,ts,n=2,w_L,w_R]     │
      │                     │                    │ encode(WHEEL_CMD 0x13)   │
      │                     │                    │ transport_->send()        │
      │                     │                    │─────────────►│            │
      │                     │                    │              │ write()───►│
      │                     │                    │              │            │ 解析帧
      │                     │                    │              │            │ 各轮独立 PID
      │◄────────────────────│◄───────────────────│ send() OK   │            │
```

---

## 4. 上行数据流程（里程计）

```
MCU 固件          ITransport(接收线程)    StreamDecoder    ChassisHal       ChassisNode      ROS2
  │                     │                     │               │                 │              │
  │ 发送 ODOMETRY 帧    │                     │               │                 │              │
  │────────────────────►│                     │               │                 │              │
  │                     │ recv(buf, n)        │               │                 │              │
  │                     │ receive_cb_(buf, n) │               │                 │              │
  │                     │────────────────────────────────────►│                 │              │
  │                     │                     │               │ onDataReceived()│              │
  │                     │                     │ feed(buf, n)  │                 │              │
  │                     │                     │◄──────────────│                 │              │
  │                     │                     │ processState() (状态机)         │              │
  │                     │                     │ ready_frames_.push_back()       │              │
  │                     │                     │               │ poll() → frame  │              │
  │                     │                     │               │──────────────►  │              │
  │                     │                     │               │ dispatchOdometryFrame()        │
  │                     │                     │               │ 解析 668B payload               │
  │                     │                     │               │ current_odom_ = odom           │
  │                     │                     │               │ odom_cb_(odom)  │              │
  │                     │                     │               │────────────────►│              │
  │                     │                     │               │                 │ onOdometry() │
  │                     │                     │               │                 │ 构造 nav_msgs │
  │                     │                     │               │                 │ odom_pub_────►│
  │                     │                     │               │                 │              │ /odom
```

---

## 5. 断线重连流程（TCP 场景）

```
重连循环线程(HAL)       TcpTransport(接收线程)       ChassisNode(wall_timer)
      │                        │                            │
      │                        │ recv() 返回 0              │
      │                        │ 对端关闭连接               │
      │                        │ connected_ = false         │
      │                        │ close(socket_fd_)          │
      │                        │                            │
      │ 检测 transport_.isConnected() == false               │
      │ connected_ = false（HAL层）                         │
      │                        │                            │
      │ sleep(backoff_ms=500)  │                            │
      │ transport_->connect()  │                            │
      │ connectSocket()        │                            │
      │ connect() 失败         │                            │
      │                        │                            │
      │ backoff_ms = 1000      │                            │
      │ sleep(1000ms)          │                            │
      │ transport_->connect()  │                            │
      │ connectSocket() 成功   │                            │
      │                        │                            │
      │ negotiator_.reset()    │                            │
      │ stream_decoder_.reset()│                            │
      │ negotiator_.sendHandshake()                        │
      │                        │  [握手过程见流程1]          │
      │ 协商成功               │                            │
      │ connected_ = true      │                            │
      │ backoff_ms = 500（重置）│                            │
      │                        │                            │
      │                        │              [5s wall_timer 触发]
      │                        │              hal->isConnected() == true
      │                        │              → 不操作      │
```

---

## 6. 急停流程

```
ROS2 spin线程              ChassisHal          ITransport        MCU 固件
      │                        │                    │                │
      │ emergencyStop("reason")│                    │                │
      │───────────────────────►│                    │                │
      │                        │ buildStopPayload() │                │
      │                        │ encode(EMERGENCY_STOP 0x12)        │
      │                        │─────────────────────────────────►  │
      │                        │                    │ write()──────►│
      │                        │                    │                │ 立即切断电机输出
      │                        │                    │                │ 设置 error_code bit15=1
      │                        │                    │                │
      │                        │                    │    CHASSIS_STATUS (is_estop=true)
      │                        │                    │◄───────────────│
      │                        │ dispatchStatusFrame()               │
      │                        │ status_.is_estop = true            │
      │                        │ status_cb_(status)                  │
      │◄──────────────────────│                    │                │
      │ onStatus(status)       │                    │                │
      │ publish /chassis_status│                    │                │
      │ is_estop:true          │                    │                │
```

---

## 7. 线程间通信汇总

```
┌─────────────────┐     receive_cb_     ┌─────────────────┐
│  Transport      │──────────────────►  │  ChassisHal     │
│  接收线程        │  (接收线程上下文)    │  onDataReceived()│
└─────────────────┘                     │                  │
                                        │ status_mutex_    │──► getStatus()     [ROS2主线程]
                                        │ odom_mutex_      │──► getOdometry()   [ROS2主线程]
                                        │ battery_mutex_   │──► getBatteryInfo()[ROS2主线程]
                                        │ cb_mutex_        │
                                        │  └─ status_cb_   │──► ChassisNode::onStatus()
                                        │  └─ odom_cb_     │──► ChassisNode::onOdometry()
                                        │  └─ battery_cb_  │──► ChassisNode::onBattery()
                                        └─────────────────┘
                                                 │
                                        send_mutex_(Transport)
                                                 │
                                        ┌────────▼────────┐
                                        │  ROS2 主线程     │
                                        │  sendVelocity()  │
                                        │  sendWheelCmd()  │
                                        └─────────────────┘
```
