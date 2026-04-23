# 06 控制器：ControlMode 双模式 + 逆运动学

## 1. 控制模式枚举

定义在 `chassis/ichassis_controller.hpp`：

```cpp
enum class ControlMode {
    BODY_VELOCITY,  // 发送体坐标速度 (vx, vy, omega) → VELOCITY_CMD (0x10)
                    // MCU 固件执行逆运动学分解，控制各轮
    WHEEL_RPM,      // 上位机执行逆运动学 → 各轮角速度 (rad/s) → WHEEL_CMD (0x13)
                    // MCU 只需对各轮独立执行 PID 速度控制
};
```

默认值：`ControlMode::BODY_VELOCITY`（兼容原有行为，零改动即可使用）。

---

## 2. 两种模式的数据流对比

### BODY_VELOCITY 模式

```
上位机                              MCU 固件
sendVelocity(vx, vy, omega)
  → buildVelocityPayload()
  → FrameCodec::encode(VELOCITY_CMD)
  → transport_->send()
                      ─────────────►  解析 VELOCITY_CMD
                                       执行逆运动学 (IK)
                                       输出各轮 PWM
```

**优点**：MCU 固件做 IK，方便在嵌入式端调整 PID 增益，无需修改上位机。

**适用**：自研 MCU 控制板，固件具备 FPU 和运动学能力。

### WHEEL_RPM 模式

```
上位机                              MCU 固件（如 ODrive/VESC）
sendVelocity(vx, vy, omega)
  → 上位机 IK (DifferentialController 或 MecanumController)
  → buildWheelPayload([w0, w1, ...])
  → FrameCodec::encode(WHEEL_CMD)
  → transport_->send()
                      ─────────────►  解析 WHEEL_CMD
                                       各轮独立 PID 速度控制
                                       输出各轮 PWM
```

**优点**：MCU 只做 PID，可使用成品 FOC 控制器；IK 参数（轮距、轮径）在上位机实时调整。

**适用**：ODrive / VESC / SimpleFOC 等成品电机控制器；需要快速迭代运动学参数。

---

## 3. `DifferentialController`

文件：`chassis/differential/differential_controller.{hpp,cpp}`

### 3.1 成员参数

```cpp
ControlMode mode_{ControlMode::BODY_VELOCITY};
double wheelbase_{0.5};        // 两轮中心距 L（米）
double wheel_radius_{0.076};   // 轮半径 r（米）
double max_linear_vel_{1.5};   // 最大线速度（m/s）
double max_angular_vel_{3.14}; // 最大角速度（rad/s）
```

### 3.2 `sendVelocity` 逻辑

```cpp
bool DifferentialController::sendVelocity(double vx, double vy, double omega) {
    vy = 0.0;   // 差速底盘：侧向速度恒为 0

    // 速度限幅
    vx    = clamp(vx,    -max_linear_vel_,  max_linear_vel_);
    omega = clamp(omega, -max_angular_vel_, max_angular_vel_);

    if (mode_ == ControlMode::WHEEL_RPM) {
        // 逆运动学
        double half_L = wheelbase_ / 2.0;
        double inv_r  = 1.0 / wheel_radius_;
        std::vector<double> wheels = {
            (vx - half_L * omega) * inv_r,   // w_L (rad/s)
            (vx + half_L * omega) * inv_r,   // w_R (rad/s)
        };
        return ChassisHal::sendWheelCmd(wheels);
    }

    return ChassisHal::sendVelocity(vx, vy, omega);
}
```

### 3.3 差速逆运动学公式推导

设体坐标系下：前向速度 `vx`，角速度 `ω`，轮距 `L`，轮半径 `r`。

地面接触点速度：

```
v_L = vx - (L/2) * ω     （左轮，转弯时走内弧）
v_R = vx + (L/2) * ω     （右轮，转弯时走外弧）
```

转换为轮角速度（rad/s）：

```
ω_L = v_L / r = (vx - L/2 * ω) / r
ω_R = v_R / r = (vx + L/2 * ω) / r
```

正值 = 前进方向旋转（俯视：右轮顺时针，左轮逆时针）。

**验证**：
- 纯前进（ω=0）：`ω_L = ω_R = vx/r` ✓
- 原地左转（vx=0, ω>0）：`ω_L = -Lω/(2r) < 0`，`ω_R = Lω/(2r) > 0` ✓（右轮正转，左轮反转）

### 3.4 配置接口

```cpp
void setControlMode(ControlMode mode);   // 切换控制模式
void setWheelbase(double meters);        // 轮距 L
void setWheelRadius(double meters);      // 轮半径 r
void setMaxLinearVel(double mps);        // 线速度上限
void setMaxAngularVel(double rps);       // 角速度上限
```

### 3.5 `getInfo()` 覆盖

```cpp
ChassisInfo DifferentialController::getInfo() const {
    ChassisInfo info = ChassisHal::getInfo();   // 取协商结果（版本、model）
    info.chassis_type = ChassisType::DIFFERENTIAL;
    info.vendor       = "Generic";
    info.model        = "DifferentialDrive";
    return info;
}
```

---

## 4. `MecanumController`

文件：`chassis/mecanum/mecanum_controller.{hpp,cpp}`

### 4.1 成员参数

```cpp
ControlMode mode_{ControlMode::BODY_VELOCITY};
double wheel_radius_{0.076};   // 轮半径 r（米）
double wheelbase_{0.4};        // 前后轴距 2l（米）
double track_{0.4};            // 左右轮距 2w（米）
double max_linear_vel_{1.5};
double max_angular_vel_{3.14};
```

轮序：`[0] FL（前左）, [1] FR（前右）, [2] RL（后左）, [3] RR（后右）`

### 4.2 `sendVelocity` 逻辑

```cpp
bool MecanumController::sendVelocity(double vx, double vy, double omega) {
    vx    = clamp(vx,    -max_linear_vel_,  max_linear_vel_);
    vy    = clamp(vy,    -max_linear_vel_,  max_linear_vel_);
    omega = clamp(omega, -max_angular_vel_, max_angular_vel_);

    if (mode_ == ControlMode::WHEEL_RPM) {
        double l     = wheelbase_ / 2.0;   // 前后半轴距
        double w     = track_     / 2.0;   // 左右半轮距
        double inv_r = 1.0 / wheel_radius_;
        std::vector<double> wheels = {
            (vx - vy - (l + w) * omega) * inv_r,   // FL
            (vx + vy + (l + w) * omega) * inv_r,   // FR
            (vx + vy - (l + w) * omega) * inv_r,   // RL
            (vx - vy + (l + w) * omega) * inv_r,   // RR
        };
        return ChassisHal::sendWheelCmd(wheels);
    }

    return ChassisHal::sendVelocity(vx, vy, omega);
}
```

### 4.3 麦克纳姆逆运动学公式推导

麦克纳姆轮运动学基于 45° 辊子角度，标准公式（车轮 ±45° 交替排布）：

```
ω_FL = (1/r) * ( vx - vy - (l+w)*ω )
ω_FR = (1/r) * ( vx + vy + (l+w)*ω )
ω_RL = (1/r) * ( vx + vy - (l+w)*ω )
ω_RR = (1/r) * ( vx - vy + (l+w)*ω )
```

其中 `l` = 前后轴半距，`w` = 左右半轮距。

**四个特殊情形验证**：

| 运动 | vx | vy | ω | FL | FR | RL | RR |
|------|----|----|---|----|----|----|----|
| 纯前进 | v | 0 | 0 | v/r | v/r | v/r | v/r | 四轮同速 ✓ |
| 纯侧移 | 0 | v | 0 | -v/r | +v/r | +v/r | -v/r | FL/RR 反转 ✓ |
| 原地旋转 | 0 | 0 | ω | -(l+w)ω/r | +(l+w)ω/r | -(l+w)ω/r | +(l+w)ω/r | 对角同号 ✓ |
| 斜45°移动 | v | v | 0 | 0 | 2v/r | 2v/r | 0 | FL/RR 不转 ✓ |

### 4.4 配置接口

```cpp
void setControlMode(ControlMode mode);
void setWheelRadius(double meters);
void setWheelbase(double meters);   // 前后轴距（2l）
void setTrack(double meters);       // 左右轮距（2w）
void setMaxLinearVel(double mps);
void setMaxAngularVel(double rps);
```

---

## 5. `sendWheelCmd()` 直接调用路径

两种控制器都直接暴露 `sendWheelCmd()` override，内部仅透传给 `ChassisHal::sendWheelCmd()`：

```cpp
bool DifferentialController::sendWheelCmd(const std::vector<double>& w) {
    return ChassisHal::sendWheelCmd(w);
}
```

上层代码可以完全绕过运动学，直接精确控制各轮：

```cpp
// 示例：直接给差速底盘左右轮分别设置不同速度（单位 rad/s）
ctrl->sendWheelCmd({12.0, 13.0});
```

---

## 6. 使用示例

### 切换到 WHEEL_RPM 模式（差速底盘）

```cpp
chassis::TransportConfig cfg;
cfg.host = "192.168.1.100"; cfg.port = 8899;
auto transport = chassis::ITransport::create(chassis::TransportType::TCP, cfg);

auto ctrl = std::make_unique<chassis::DifferentialController>(std::move(transport));
ctrl->setControlMode(chassis::ControlMode::WHEEL_RPM);
ctrl->setWheelRadius(0.076);    // 76mm 轮
ctrl->setWheelbase(0.5);        // 500mm 轮距
ctrl->setMaxLinearVel(1.5);
ctrl->setMaxAngularVel(3.14);

if (!ctrl->connect()) { /* 处理失败 */ }

// 之后正常调用 sendVelocity，内部自动做 IK 并发 WHEEL_CMD
ctrl->sendVelocity(0.5, 0.0, 0.3);   // → WHEEL_CMD (0x13)
```

### 切换到 WHEEL_RPM 模式（麦克纳姆底盘）

```cpp
auto ctrl = std::make_unique<chassis::MecanumController>(std::move(transport));
ctrl->setControlMode(chassis::ControlMode::WHEEL_RPM);
ctrl->setWheelRadius(0.076);
ctrl->setWheelbase(0.4);   // 前后轴距 400mm
ctrl->setTrack(0.4);       // 左右轮距 400mm

ctrl->sendVelocity(0.3, 0.2, 0.0);   // → WHEEL_CMD，IK 在上位机完成
```

---

## 7. 模式选择指南

| 考量 | 选 BODY_VELOCITY | 选 WHEEL_RPM |
|------|-----------------|-------------|
| MCU 能力 | MCU 有 FPU，固件已实现 IK | MCU 是成品 FOC 控制器，只做 PID |
| 调参灵活性 | 需更新 MCU 固件 | 只需改上位机参数（轮径、轮距） |
| 实时性要求 | 低延迟（IK 在 MCU 上，无额外网络 RTT） | 上位机 IK 耗时可忽略（< 1μs） |
| 多路线速度同步 | MCU 统一输出，同步好 | 上位机打包一帧 WHEEL_CMD，也是原子发送 |
| 协议兼容性 | 旧版 MCU 固件也支持 | 需要 MCU 支持 0x13 帧 |
