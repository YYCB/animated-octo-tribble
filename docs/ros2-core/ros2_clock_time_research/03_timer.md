# Clock/Time（三）：Timer 源码深度解析

## 一、create_timer() 调用链

```cpp
// rclcpp/include/rclcpp/node_impl.hpp

template<typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::TimerBase::SharedPtr
Node::create_timer(
  std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT callback,
  rclcpp::CallbackGroup::SharedPtr group)
{
  return rclcpp::create_timer(
    this,
    get_clock(),    // ★ RCL_ROS_CLOCK（支持 sim time）
    rclcpp::Duration(period),
    std::move(callback),
    group
  );
}

// rclcpp/include/rclcpp/create_timer.hpp
template<typename NodeT, typename CallbackT>
typename rclcpp::TimerBase::SharedPtr
create_timer(
  NodeT && node,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Duration period,
  CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group)
{
  // ★ 构造 GenericTimer<rclcpp::Clock>
  auto timer = rclcpp::GenericTimer<CallbackT>::make_shared(
    clock,
    period,
    std::forward<CallbackT>(callback),
    node->get_node_base_interface()->get_context()
  );
  
  // ★ 将 Timer 注册到 NodeTimers（绑定 CallbackGroup）
  node->get_node_timers_interface()->add_timer(timer, group);
  
  return timer;
}
```

---

## 二、GenericTimer / TimerBase 实现

```cpp
// rclcpp/include/rclcpp/timer.hpp

class TimerBase
{
public:
  TimerBase(
    rclcpp::Clock::SharedPtr clock,
    std::chrono::nanoseconds period,
    rclcpp::Context::SharedPtr context)
  : clock_(clock), timer_handle_(new rcl_timer_t)
  {
    // ★ 初始化 rcl_timer
    *timer_handle_ = rcl_get_zero_initialized_timer();
    
    rcl_ret_t ret = rcl_timer_init(
      timer_handle_.get(),
      clock->get_clock_handle(),  // ★ 绑定到节点时钟
      context->get_rcl_context().get(),
      period.count(),             // 周期（纳秒）
      nullptr,                    // 不使用 rcl 层回调（用 rclcpp Executor）
      rcl_get_default_allocator()
    );
  }
  
  // ★ 检查 Timer 是否到期（Executor 调用）
  bool is_ready() const
  {
    bool is_ready;
    rcl_timer_is_ready(timer_handle_.get(), &is_ready);
    return is_ready;
  }
  
  // ★ 重置 Timer（执行后调用，重新开始计时）
  void reset()
  {
    rcl_timer_reset(timer_handle_.get());
  }
  
  // ★ 取消 Timer（不再触发）
  void cancel()
  {
    rcl_timer_cancel(timer_handle_.get());
  }
  
  bool is_canceled() const;
  
  // ★ 获取距下次触发的时间
  std::chrono::nanoseconds time_until_trigger()
  {
    int64_t time_until;
    rcl_timer_get_time_until_next_call(timer_handle_.get(), &time_until);
    return std::chrono::nanoseconds(time_until);
  }

protected:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rcl_timer_t> timer_handle_;
};

template<typename CallbackT>
class GenericTimer : public TimerBase
{
public:
  GenericTimer(
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Duration period,
    CallbackT && callback,
    rclcpp::Context::SharedPtr context)
  : TimerBase(clock, period, context),
    callback_(std::forward<CallbackT>(callback))
  {}
  
  // ★ Executor 调用 execute_callback()
  void execute_callback()
  {
    // 调用 rcl_timer_call()（更新 last_call_time）
    rcl_timer_call(timer_handle_.get());
    
    // 调用用户 callback
    if constexpr (std::is_invocable_v<CallbackT, TimerBase::SharedPtr>) {
      // 带 timer 参数的回调
      callback_(shared_from_this());
    } else {
      // 无参数回调（最常见）
      callback_();
    }
  }

private:
  CallbackT callback_;
};
```

---

## 三、rcl_timer_init() 内部

```c
// rcl/src/rcl/timer.c

rcl_ret_t
rcl_timer_init(
  rcl_timer_t * timer,
  rcl_clock_t * clock,
  rcl_context_t * context,
  int64_t period,
  const rcl_timer_callback_t callback,
  rcl_allocator_t allocator)
{
  // ★ 记录创建时间
  int64_t now;
  rcl_clock_get_now(clock, &now);
  timer->impl->last_call_time = now;
  timer->impl->next_call_time = now + period;
  
  // ★ 如果是 ROS_CLOCK，注册 time_jump 回调
  if (clock->type == RCL_ROS_CLOCK) {
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = true;
    threshold.min_forward.nanoseconds = 0;  // 任何向前跳跃
    threshold.min_backward.nanoseconds = -1; // 任何向后跳跃
    
    rcl_clock_add_jump_callback(
      clock,
      threshold,
      _rcl_timer_time_jump,  // 回调函数
      timer                  // user_data
    );
  }
}
```

---

## 四、rcl_timer_is_ready() — 到期检查

```c
// rcl/src/rcl/timer.c

rcl_ret_t
rcl_timer_is_ready(const rcl_timer_t * timer, bool * is_ready)
{
  int64_t now;
  rcl_clock_get_now(timer->impl->clock, &now);
  
  // ★ 当前时间 >= 下次触发时间 → 已到期
  *is_ready = !timer->impl->canceled &&
              (now >= timer->impl->next_call_time);
  
  return RCL_RET_OK;
}

// rcl_timer_call()：记录调用时间，更新下次触发点
rcl_ret_t
rcl_timer_call(rcl_timer_t * timer)
{
  int64_t now;
  rcl_clock_get_now(timer->impl->clock, &now);
  
  timer->impl->last_call_time = now;
  // ★ 从 last_call_time 开始计算下次触发（而非从 now 开始）
  // 这确保了定时精度（补偿处理延迟）
  timer->impl->next_call_time = timer->impl->last_call_time + timer->impl->period;
  
  return RCL_RET_OK;
}
```

---

## 五、rclcpp::Rate — 循环频率控制

```cpp
// rclcpp/include/rclcpp/rate.hpp

class Rate
{
public:
  // ★ 使用 ROS 时钟（支持 sim time）
  explicit Rate(double rate, rclcpp::Clock::SharedPtr clock = nullptr)
  : period_(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate))),
    clock_(clock ? clock : std::make_shared<rclcpp::Clock>(RCL_ROS_CLOCK))
  {
    last_interval_ = clock_->now();
  }
  
  // ★ 睡眠直到下次应触发的时间
  bool sleep()
  {
    rclcpp::Time expected_end = last_interval_ + period_;
    rclcpp::Time actual_end = clock_->now();
    
    if (actual_end < expected_end) {
      // 还有时间，睡眠
      clock_->sleep_until(expected_end);
      actual_end = clock_->now();
    }
    
    // 更新下次触发基准
    last_interval_ += period_;
    
    // 返回：是否准点（实际时间 == 期望时间）
    return actual_end < (last_interval_ + period_);
  }

private:
  std::chrono::nanoseconds period_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_interval_;
};

// 使用：
rclcpp::Rate rate(100.0);  // 100Hz
while (rclcpp::ok()) {
  // 处理逻辑...
  rate.sleep();  // ★ 控制循环频率
}

// WallRate：强制使用 STEADY_CLOCK
class WallRate : public Rate {
  explicit WallRate(double rate)
  : Rate(rate, std::make_shared<rclcpp::Clock>(RCL_STEADY_CLOCK)) {}
};
```

---

## 六、Executor 中的 Timer 处理

```cpp
// rclcpp/src/rclcpp/executor.cpp

void Executor::execute_timer(rclcpp::TimerBase::SharedPtr timer)
{
  // ★ 直接调用 execute_callback（已在 is_ready() 检查通过）
  timer->execute_callback();
}

// wait_set 中 Timer 的处理：
// rcl_wait() 使用 timerfd（Linux）或类似机制等待 Timer 到期
// → 减少空轮询开销
// → 注意：ROS_CLOCK Timer 在 sim time 下不使用 timerfd
//         而是每次 /clock 更新时检查所有 Timer 是否到期
```
