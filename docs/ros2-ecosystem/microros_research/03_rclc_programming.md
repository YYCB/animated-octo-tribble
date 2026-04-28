# rclc 编程模式

## 一、rclc_executor 深度解析

### 1.1 Executor 设计原理

`rclc_executor_t` 是 micro-ROS 的核心调度器，设计为**单线程、零动态分配**的事件处理引擎，适配 RTOS 的任务模型（与 rclcpp 的多线程 Executor 形成对比）。

```
rclc_executor_spin_some() 调用流程：
  ↓
  对所有注册实体遍历 wait_set
  ↓
  rcl_wait()（超时 = timeout_ns）
  ↓
  对就绪实体依次调用：
  - rcl_take_with_info()（Subscription）
  - rcl_timer_call()（Timer）
  - rcl_take_request()（Service）
  - rcl_take_response()（Client）
  ↓
  执行对应回调
  ↓
  返回（非阻塞，可重入）
```

### 1.2 初始化与注册

```c
// 步骤 1：确定 handles 数量（所有注册实体的总数）
// handles = subscription 数 + timer 数 + service 数 + client 数 + parameter_server entities
#define NUM_HANDLES 5

rclc_executor_t executor;
rclc_executor_init(&executor, &support.context, NUM_HANDLES, &allocator);

// 步骤 2：注册 Subscription（两种触发模式）
rclc_executor_add_subscription(
    &executor,
    &joint_cmd_sub,
    &joint_cmd_msg,
    joint_command_callback,
    ON_NEW_DATA);    // 仅有新数据时调用（推荐：减少无效调用）

rclc_executor_add_subscription(
    &executor,
    &heartbeat_sub,
    &heartbeat_msg,
    heartbeat_callback,
    ALWAYS);         // 每次 spin 都调用（用于超时检测）

// 步骤 3：注册 Timer
rclc_executor_add_timer(&executor, &publish_timer);

// 步骤 4：注册 Service
rclc_executor_add_service(
    &executor, &config_service,
    &config_request, &config_response,
    config_service_callback);
```

### 1.3 与 FreeRTOS Task 的集成模式

```c
// 推荐模式 A：专用 micro-ROS Task（独占 CPU 时间片）
void micro_ros_task(void *arg) {
    // ... 初始化（略）...

    // 主循环
    for (;;) {
        // spin_some：处理所有就绪事件后立即返回
        // timeout_ns = 10ms：等待新事件的最长时间
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // 让出调度器（避免饿死其他任务）
        // 若 spin_some 在 timeout 内提前返回（有事件），taskYIELD 确保公平调度
        taskYIELD();
    }
}

// 任务创建（16KB 栈：micro-ROS 需要较大栈空间）
xTaskCreate(micro_ros_task, "uros",
            configMINIMAL_STACK_SIZE * 8,  // ~16384 bytes
            NULL,
            tskIDLE_PRIORITY + 5,          // 中等优先级
            &uros_task_handle);
```

```c
// 推荐模式 B：Supervisor 模式（断线重连）
void micro_ros_task(void *arg) {
    for (;;) {
        // 尝试连接 Agent
        if (!connect_to_agent()) {
            vTaskDelay(pdMS_TO_TICKS(500));  // 500ms 后重试
            continue;
        }

        // 成功连接：进入正常 spin 循环
        while (ping_agent_ok()) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        }

        // 断线：清理实体，准备重连
        destroy_ros_entities();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 检查 Agent 连接状态（发送 PING，等待 PONG）
bool ping_agent_ok() {
    return rmw_uros_ping_agent(100, 3) == RMW_RET_OK;
    // timeout_ms=100, attempts=3
}
```

---

## 二、Timer（定时器）

### 2.1 初始化与精度

```c
// 500 Hz 定时器（2ms 周期）
rcl_timer_t publish_timer;
rclc_timer_init_default(
    &publish_timer,
    &support,
    RCL_MS_TO_NS(2),        // period_ns = 2,000,000 ns
    publish_joint_states_cb);  // 回调函数

// 回调函数签名
void publish_joint_states_cb(rcl_timer_t * timer, int64_t last_call_time) {
    if (timer == NULL) return;

    // 读取编码器（具体实现依赖硬件）
    for (int i = 0; i < NUM_JOINTS; i++) {
        joint_state.position.data[i] = read_encoder(i);
        joint_state.velocity.data[i] = compute_velocity(i);
    }
    joint_state.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    joint_state.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1e6;

    rcl_publish(&publisher, &joint_state, NULL);
}
```

### 2.2 定时精度与 RTOS Tick 的关系

FreeRTOS 的 Tick 周期（`configTICK_RATE_HZ`，通常 1000 Hz = 1ms）决定了定时器的**最小精度**：

| configTICK_RATE_HZ | Tick 周期 | rclc Timer 最小精度 | 适用控制频率 |
|---------------------|----------|-------------------|------------|
| 100 Hz | 10 ms | 10 ms | ≤ 50 Hz |
| 1000 Hz（推荐） | 1 ms | 1 ms | ≤ 500 Hz |
| 10000 Hz（高精度） | 0.1 ms | 0.1 ms | ≤ 2 kHz（需要 hrtimer 支持） |

```c
// FreeRTOSConfig.h — 推荐配置
#define configTICK_RATE_HZ              1000   // 1ms tick
#define configUSE_TICKLESS_IDLE         0      // 关闭 tickless idle（保持定时精度）
#define configUSE_16_BIT_TICKS          0      // 32-bit tick counter
```

> **注意**：FreeRTOS 的 Timer 精度受 `portTICK_PERIOD_MS`（1ms）限制。若需要 < 1ms 精度（如 4 kHz 力控），应使用 **硬件定时器中断** + 双缓冲，让中断处理硬实时任务，rclc Timer 仅负责数据发布。

### 2.3 高精度定时器 + micro-ROS 混合模式

```c
// 硬件定时器 ISR（4 kHz，每 250μs 触发）
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        // 硬实时：直接在 ISR 中读传感器，写 PWM
        uint16_t adc_val = DMA_ADC_value;
        float force = adc_to_newton(adc_val);

        // 写入双缓冲（FreeRTOS 安全访问）
        force_buffer[write_idx] = force;
        write_idx ^= 1;

        // 触发 micro-ROS 发布任务（信号量通知，不在 ISR 中发布）
        BaseType_t higher_priority_woken = pdFALSE;
        xSemaphoreGiveFromISR(publish_sem, &higher_priority_woken);
        portYIELD_FROM_ISR(higher_priority_woken);
    }
}

// micro-ROS 发布任务（降频：4 kHz → 500 Hz 发布）
void publish_force_task(void *arg) {
    static uint8_t decimation = 0;
    for (;;) {
        xSemaphoreTake(publish_sem, portMAX_DELAY);
        if (++decimation >= 8) {  // 4000/8 = 500 Hz
            decimation = 0;
            wrench_msg.wrench.force.z = force_buffer[read_idx];
            rcl_publish(&force_publisher, &wrench_msg, NULL);
        }
    }
}
```

---

## 三、Service / Client

### 3.1 Service 端（MCU 处理主机请求）

```c
// 定义消息实例（全局或静态分配，避免 MCU 栈溢出）
std_srvs__srv__SetBool_Request  enable_request;
std_srvs__srv__SetBool_Response enable_response;

// 初始化 Service
rcl_service_t enable_torque_service;
rclc_service_init_default(
    &enable_torque_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "/enable_torque");

// 注册到 Executor
rclc_executor_add_service(
    &executor,
    &enable_torque_service,
    &enable_request,
    &enable_response,
    enable_torque_callback);

// 回调实现
void enable_torque_callback(
    const void * request_msg,
    void * response_msg)
{
    std_srvs__srv__SetBool_Request * req =
        (std_srvs__srv__SetBool_Request *) request_msg;
    std_srvs__srv__SetBool_Response * resp =
        (std_srvs__srv__SetBool_Response *) response_msg;

    if (req->data) {
        // 使能力矩
        set_motor_enable(true);
        resp->success = true;
        resp->message.data = "Torque enabled";
    } else {
        set_motor_enable(false);
        resp->success = true;
        resp->message.data = "Torque disabled";
    }
}
```

### 3.2 Client 端（MCU 主动请求主机服务，较少见）

```c
// MCU 请求主机端的时间同步服务
rcl_client_t time_sync_client;
rclc_client_init_default(
    &time_sync_client,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(builtin_interfaces, srv, Time),
    "/time_sync");

// 发送请求
builtin_interfaces__srv__Time_Request req;
int64_t seq_num;
rcl_send_request(&time_sync_client, &req, &seq_num);
```

---

## 四、参数服务器

### 4.1 完整参数服务器配置

```c
// 参数服务器需要占用较多 Executor handles（通常 +10）
#define PARAM_SERVER_HANDLES RCLC_PARAMETER_SERVER_EXECUTOR_HANDLES_NUMBER

rclc_parameter_server_t param_server;
// executor 需要为参数服务器预留足够的 handles
rclc_executor_init(&executor, &support.context,
                   NUM_HANDLES + PARAM_SERVER_HANDLES, &allocator);

rclc_parameter_server_init_default(&param_server, &node, &executor);

// 添加参数
rclc_add_parameter(&param_server, "control_mode",   RCLC_PARAMETER_INT);
rclc_add_parameter(&param_server, "kp",             RCLC_PARAMETER_DOUBLE);
rclc_add_parameter(&param_server, "ki",             RCLC_PARAMETER_DOUBLE);
rclc_add_parameter(&param_server, "kd",             RCLC_PARAMETER_DOUBLE);
rclc_add_parameter(&param_server, "max_velocity",   RCLC_PARAMETER_DOUBLE);
rclc_add_parameter(&param_server, "enabled",        RCLC_PARAMETER_BOOL);

// 设置初始值
rclc_parameter_set_int(&param_server,    "control_mode",  1);
rclc_parameter_set_double(&param_server, "kp",            10.0);
rclc_parameter_set_double(&param_server, "ki",            0.1);
rclc_parameter_set_double(&param_server, "kd",            0.5);
rclc_parameter_set_double(&param_server, "max_velocity",  3.14);
rclc_parameter_set_bool(&param_server,   "enabled",       true);

// 参数变更回调（动态参数更新）
bool on_parameter_changed(const Parameter * param,
                           const Parameter * old_param,
                           void * context)
{
    if (strcmp(param->name.data, "kp") == 0) {
        pid_gains.kp = param->value.double_value;
        return true;  // 接受变更
    }
    if (strcmp(param->name.data, "enabled") == 0) {
        set_motor_enable(param->value.bool_value);
        return true;
    }
    // 拒绝 control_mode 在运行时变更
    if (strcmp(param->name.data, "control_mode") == 0) {
        return false;  // 拒绝：返回 false
    }
    return true;
}

rclc_executor_add_parameter_server_with_context(
    &executor, &param_server, on_parameter_changed, NULL);
```

### 4.2 主机端动态调参（ros2 param 命令）

```bash
# 查看 MCU 节点的参数
ros2 param list /joint_controller

# 动态修改 PID 增益
ros2 param set /joint_controller kp 15.0
ros2 param set /joint_controller enabled false

# 保存参数到 YAML（用于 OTA 预配置）
ros2 param dump /joint_controller --output-dir ~/params/
```

---

## 五、生命周期节点（rclc_lifecycle）

### 5.1 rclc Lifecycle 与 ROS 2 lifecycle_node 的兼容性

rclc 支持 ROS 2 标准生命周期状态机（`unconfigured → inactive → active → finalized`），可被主机端 `lifecycle_manager` 统一管理：

```c
#include <rclc_lifecycle/rclc_lifecycle.h>

// 创建 lifecycle 节点
rcl_node_t node;
rclc_node_init_default(&node, "joint_controller", "robot1", &support);

rclc_lifecycle_node_t lifecycle_node;
rcl_lifecycle_state_machine_t state_machine =
    rcl_lifecycle_get_zero_initialized_state_machine();

rclc_lifecycle_node_init_default(
    &lifecycle_node, &node, &state_machine, &support, true);

// 注册生命周期回调
rclc_lifecycle_register_on_configure(&lifecycle_node, &on_configure);
rclc_lifecycle_register_on_activate(&lifecycle_node, &on_activate);
rclc_lifecycle_register_on_deactivate(&lifecycle_node, &on_deactivate);
rclc_lifecycle_register_on_cleanup(&lifecycle_node, &on_cleanup);

// 注册到 Executor
rclc_executor_add_lifecycle_node(&executor, &lifecycle_node);
```

```c
// 状态回调实现
rcl_ret_t on_configure(const State * state) {
    // 初始化硬件（编码器、PWM、ADC）
    encoder_init();
    pwm_init();
    // 创建 publishers/subscribers（在 configure 而非 init 中创建）
    create_ros_entities();
    return RCL_RET_OK;
}

rcl_ret_t on_activate(const State * state) {
    // 使能电机控制环
    set_motor_enable(true);
    return RCL_RET_OK;
}

rcl_ret_t on_deactivate(const State * state) {
    // 停止电机（保持上电，可快速重激活）
    set_motor_enable(false);
    return RCL_RET_OK;
}
```

### 5.2 主机端 lifecycle 管理

```bash
# 通过 ros2 lifecycle 命令控制 MCU 节点状态
ros2 lifecycle set /joint_controller configure
ros2 lifecycle set /joint_controller activate
ros2 lifecycle set /joint_controller deactivate

# 使用 nav2_lifecycle_manager 批量管理
ros2 launch lifecycle_manager lifecycle_manager.launch.py \
    node_names:="['/joint_controller', '/imu_node']"
```

---

## 六、内存策略

### 6.1 Static vs Dynamic 内存管理

micro-ROS 在 MCU 上的内存管理是**最关键的工程挑战**之一：

| 策略 | 适用场景 | 优点 | 缺点 |
|------|---------|------|------|
| **Static**（全局/栈分配） | 生产环境 | 确定性，无碎片，无 `malloc` | 需提前知道最大消息大小 |
| **Dynamic**（`pvPortMalloc`） | 原型验证 | 灵活，代码简洁 | 可能碎片化；RTOS heap 有限 |

### 6.2 Static 内存策略实现

```c
// 方案：使用 micro_ros_utilities 的静态内存辅助宏
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// 为 JointState（6 关节）静态分配内存
#define NUM_JOINTS 6
#define JOINT_NAME_MAX_LEN 16

sensor_msgs__msg__JointState joint_state_msg;

// 静态分配 name 字符串数组
static char joint_name_buffers[NUM_JOINTS][JOINT_NAME_MAX_LEN];
static rosidl_runtime_c__String joint_names[NUM_JOINTS];

// 静态分配数值数组
static double position_data[NUM_JOINTS];
static double velocity_data[NUM_JOINTS];
static double effort_data[NUM_JOINTS];

void init_joint_state_msg() {
    // 设置 name 字段
    joint_state_msg.name.data = joint_names;
    joint_state_msg.name.size = NUM_JOINTS;
    joint_state_msg.name.capacity = NUM_JOINTS;

    for (int i = 0; i < NUM_JOINTS; i++) {
        snprintf(joint_name_buffers[i], JOINT_NAME_MAX_LEN, "joint%d", i + 1);
        joint_names[i].data     = joint_name_buffers[i];
        joint_names[i].size     = strlen(joint_name_buffers[i]);
        joint_names[i].capacity = JOINT_NAME_MAX_LEN;
    }

    // 设置数值字段（直接指向静态数组）
    joint_state_msg.position.data     = position_data;
    joint_state_msg.position.size     = NUM_JOINTS;
    joint_state_msg.position.capacity = NUM_JOINTS;

    joint_state_msg.velocity.data     = velocity_data;
    joint_state_msg.velocity.size     = NUM_JOINTS;
    joint_state_msg.velocity.capacity = NUM_JOINTS;

    joint_state_msg.effort.data     = effort_data;
    joint_state_msg.effort.size     = NUM_JOINTS;
    joint_state_msg.effort.capacity = NUM_JOINTS;
}
```

### 6.3 FreeRTOS pvPortMalloc 集成

当必须使用动态分配时，将 `rcl_allocator_t` 重定向到 FreeRTOS 堆：

```c
// 自定义 allocator，使用 FreeRTOS heap
static void * freertos_allocate(size_t size, void * state) {
    (void)state;
    return pvPortMalloc(size);
}

static void freertos_deallocate(void * pointer, void * state) {
    (void)state;
    vPortFree(pointer);
}

static void * freertos_reallocate(void * pointer, size_t size, void * state) {
    (void)state;
    // FreeRTOS 无原生 realloc，手动实现
    if (pointer == NULL) return pvPortMalloc(size);
    void * new_ptr = pvPortMalloc(size);
    if (new_ptr != NULL) {
        // 注意：无法知道原始大小，需在分配时记录
        memcpy(new_ptr, pointer, size);
        vPortFree(pointer);
    }
    return new_ptr;
}

static void * freertos_zero_allocate(size_t n, size_t s, void * state) {
    (void)state;
    size_t total = n * s;
    void * ptr = pvPortMalloc(total);
    if (ptr != NULL) memset(ptr, 0, total);
    return ptr;
}

// 配置 allocator
rcl_allocator_t freertos_allocator = {
    .allocate      = freertos_allocate,
    .deallocate    = freertos_deallocate,
    .reallocate    = freertos_reallocate,
    .zero_allocate = freertos_zero_allocate,
    .state         = NULL,
};
```

---

## 七、完整示例：FreeRTOS + micro-ROS 关节控制器

以下是一个功能完整的 6 轴关节控制器伪代码，整合了以上所有知识点：

```c
/**
 * 文件：joint_controller_microros.c
 * 目标硬件：STM32H7（6轴串联机械臂）
 * RTOS：FreeRTOS（configTICK_RATE_HZ = 1000）
 * 传输：UART2 Serial，921600 bps，DMA
 * 功能：
 *   - 500 Hz 读编码器 → publish /joint_states
 *   - 订阅 /joint_commands → 输出 PWM / 电流指令
 *   - 参数服务器：PID 增益动态调参
 *   - Service：/enable_torque（启用/禁用力矩）
 *   - Lifecycle 节点：支持远程状态管理
 */

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_lifecycle/rclc_lifecycle.h>
#include <rclc_parameter/rclc_parameter.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_srvs/srv/set_bool.h>

#define NUM_JOINTS 6
#define PUBLISH_HZ 500
#define PUBLISH_PERIOD_MS (1000 / PUBLISH_HZ)

// ─── 全局 ROS 实体（静态分配）─────────────────────────────────────────────────
static rcl_allocator_t      allocator;
static rclc_support_t       support;
static rcl_node_t           node;
static rclc_executor_t      executor;
static rclc_parameter_server_t param_server;

static rcl_publisher_t      joint_state_pub;
static rcl_subscription_t   joint_cmd_sub;
static rcl_timer_t          publish_timer;
static rcl_service_t        enable_svc;

// ─── 消息内存（静态分配）─────────────────────────────────────────────────────
static sensor_msgs__msg__JointState         js_msg;
static std_msgs__msg__Float64MultiArray     cmd_msg;
static std_srvs__srv__SetBool_Request       enable_req;
static std_srvs__srv__SetBool_Response      enable_resp;

// 关节名称静态缓冲
static char      name_bufs[NUM_JOINTS][16];
static rosidl_runtime_c__String names[NUM_JOINTS];
static double    pos[NUM_JOINTS], vel[NUM_JOINTS], eff[NUM_JOINTS];
static double    cmd_data[NUM_JOINTS];

// ─── PID 状态（每轴）─────────────────────────────────────────────────────────
static struct {
    double kp, ki, kd;
    double integral, prev_error;
} pid[NUM_JOINTS];

// ─── 电机使能标志 ─────────────────────────────────────────────────────────────
static volatile bool motor_enabled = false;

// ─── 静态消息初始化 ──────────────────────────────────────────────────────────
static void init_messages(void) {
    // JointState
    for (int i = 0; i < NUM_JOINTS; i++) {
        snprintf(name_bufs[i], sizeof(name_bufs[i]), "joint%d", i + 1);
        names[i].data     = name_bufs[i];
        names[i].size     = strlen(name_bufs[i]);
        names[i].capacity = sizeof(name_bufs[i]);
    }
    js_msg.name.data     = names;
    js_msg.name.size     = NUM_JOINTS;
    js_msg.name.capacity = NUM_JOINTS;
    js_msg.position.data     = pos; js_msg.position.size = js_msg.position.capacity = NUM_JOINTS;
    js_msg.velocity.data     = vel; js_msg.velocity.size = js_msg.velocity.capacity = NUM_JOINTS;
    js_msg.effort.data       = eff; js_msg.effort.size   = js_msg.effort.capacity   = NUM_JOINTS;

    // JointCommand
    cmd_msg.data.data     = cmd_data;
    cmd_msg.data.size     = NUM_JOINTS;
    cmd_msg.data.capacity = NUM_JOINTS;
}

// ─── 定时器回调：读编码器并发布 ─────────────────────────────────────────────
static void publish_joint_states(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (!timer) return;

    uint64_t ts_ms = rmw_uros_epoch_millis();
    js_msg.header.stamp.sec      = ts_ms / 1000;
    js_msg.header.stamp.nanosec  = (ts_ms % 1000) * 1000000UL;

    for (int i = 0; i < NUM_JOINTS; i++) {
        pos[i] = encoder_read_rad(i);
        vel[i] = encoder_read_rad_s(i);
        eff[i] = current_sense_nm(i);
    }

    rcl_publish(&joint_state_pub, &js_msg, NULL);
}

// ─── Subscription 回调：接收关节命令 ────────────────────────────────────────
static void joint_command_callback(const void *msg) {
    const std_msgs__msg__Float64MultiArray *cmd =
        (const std_msgs__msg__Float64MultiArray *)msg;

    if (!motor_enabled) return;
    if (cmd->data.size < NUM_JOINTS) return;

    for (int i = 0; i < NUM_JOINTS; i++) {
        double target = cmd->data.data[i];
        double error  = target - pos[i];

        pid[i].integral   += error * 0.002;  // dt = 2ms
        double derivative  = (error - pid[i].prev_error) / 0.002;
        pid[i].prev_error  = error;

        double output = pid[i].kp * error
                      + pid[i].ki * pid[i].integral
                      + pid[i].kd * derivative;

        pwm_set_duty(i, output);  // 输出到电机驱动 PWM
    }
}

// ─── Service 回调：使能/禁用力矩 ────────────────────────────────────────────
static void enable_torque_callback(const void *req, void *resp) {
    const std_srvs__srv__SetBool_Request  *rq = req;
          std_srvs__srv__SetBool_Response *rs = resp;
    motor_enabled    = rq->data;
    rs->success      = true;
    rs->message.data = rq->data ? "Torque ON" : "Torque OFF";
    rs->message.size = rs->message.capacity = strlen(rs->message.data);
    if (!motor_enabled) {
        for (int i = 0; i < NUM_JOINTS; i++) pwm_set_duty(i, 0.0);
    }
}

// ─── 参数变更回调 ─────────────────────────────────────────────────────────────
static bool on_param_changed(const Parameter *p, const Parameter *old_p, void *ctx) {
    (void)old_p; (void)ctx;
    for (int i = 0; i < NUM_JOINTS; i++) {
        char kp_name[16], ki_name[16], kd_name[16];
        snprintf(kp_name, sizeof(kp_name), "j%d_kp", i);
        snprintf(ki_name, sizeof(ki_name), "j%d_ki", i);
        snprintf(kd_name, sizeof(kd_name), "j%d_kd", i);
        if (!strcmp(p->name.data, kp_name)) { pid[i].kp = p->value.double_value; return true; }
        if (!strcmp(p->name.data, ki_name)) { pid[i].ki = p->value.double_value; return true; }
        if (!strcmp(p->name.data, kd_name)) { pid[i].kd = p->value.double_value; return true; }
    }
    return true;
}

// ─── ROS 实体创建/销毁 ────────────────────────────────────────────────────────
static bool create_entities(void) {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "joint_controller", "robot1", &support) != RCL_RET_OK) return false;

    // Publisher
    rclc_publisher_init_default(&joint_state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/robot1/joint_states");

    // Subscriber
    rclc_subscription_init_default(&joint_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/robot1/joint_commands");

    // Timer
    rclc_timer_init_default(&publish_timer, &support,
        RCL_MS_TO_NS(PUBLISH_PERIOD_MS), publish_joint_states);

    // Service
    rclc_service_init_default(&enable_svc, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/robot1/enable_torque");

    // Executor（handles = sub + timer + service + param_server）
    #define N_HANDLES (1 + 1 + 1 + RCLC_PARAMETER_SERVER_EXECUTOR_HANDLES_NUMBER)
    rclc_executor_init(&executor, &support.context, N_HANDLES, &allocator);
    rclc_executor_add_subscription(&executor, &joint_cmd_sub, &cmd_msg,
                                    joint_command_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &publish_timer);
    rclc_executor_add_service(&executor, &enable_svc,
                               &enable_req, &enable_resp, enable_torque_callback);

    // 参数服务器
    rclc_parameter_server_init_default(&param_server, &node, &executor);
    for (int i = 0; i < NUM_JOINTS; i++) {
        char name[16];
        snprintf(name, sizeof(name), "j%d_kp", i); rclc_add_parameter(&param_server, name, RCLC_PARAMETER_DOUBLE); rclc_parameter_set_double(&param_server, name, 10.0); pid[i].kp = 10.0;
        snprintf(name, sizeof(name), "j%d_ki", i); rclc_add_parameter(&param_server, name, RCLC_PARAMETER_DOUBLE); rclc_parameter_set_double(&param_server, name, 0.1);  pid[i].ki = 0.1;
        snprintf(name, sizeof(name), "j%d_kd", i); rclc_add_parameter(&param_server, name, RCLC_PARAMETER_DOUBLE); rclc_parameter_set_double(&param_server, name, 0.5);  pid[i].kd = 0.5;
    }
    rclc_executor_add_parameter_server_with_context(&executor, &param_server, on_param_changed, NULL);

    // 时间同步（与 Agent 时钟同步）
    rmw_uros_sync_session(1000);

    return true;
}

static void destroy_entities(void) {
    rcl_subscription_fini(&joint_cmd_sub, &node);
    rcl_publisher_fini(&joint_state_pub, &node);
    rcl_timer_fini(&publish_timer);
    rcl_service_fini(&enable_svc, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ─── FreeRTOS 主任务 ──────────────────────────────────────────────────────────
void micro_ros_task(void *arg) {
    (void)arg;
    set_microros_serial_transports(huart2);  // 配置 UART2 DMA 传输
    init_messages();

    enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state = WAITING_AGENT;

    for (;;) {
        switch (state) {
        case WAITING_AGENT:
            // 每 100ms 尝试 ping Agent，成功后进入 AVAILABLE
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                state = AGENT_AVAILABLE;
            else
                vTaskDelay(pdMS_TO_TICKS(100));
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) state = AGENT_CONNECTED;
            else                    state = WAITING_AGENT;
            break;

        case AGENT_CONNECTED:
            // 正常工作状态：spin + 心跳检测
            if (rmw_uros_ping_agent(0, 0) == RMW_RET_OK) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            } else {
                state = AGENT_DISCONNECTED;
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            motor_enabled = false;
            for (int i = 0; i < NUM_JOINTS; i++) pwm_set_duty(i, 0.0);
            state = WAITING_AGENT;
            break;
        }
    }
}
```
