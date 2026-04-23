# Launch 系统（三）：事件系统详解

## 一、事件系统架构

```
事件（Event）：系统状态变化的通知
事件处理器（EventHandler）：响应特定类型事件的 Action 集合

事件流：
  ExecuteProcess 启动 → ProcessStarted 事件
  进程退出         → ProcessExited 事件
  用户 Ctrl+C      → Shutdown 事件
  
  RegisterEventHandler(OnProcessExit(...)) 注册处理器
  → LaunchService 主循环检测事件
  → 找到匹配处理器
  → 执行处理器中的 Actions
```

---

## 二、内置事件类型

```python
# launch/launch/events/ 目录下

# ★ 进程生命周期事件
class ProcessStarted(Event):
    """进程成功启动"""
    NAME = 'launch.events.process.ProcessStarted'
    def __init__(self, *, action, pid, cmd, ...):
        self.action = action  # ExecuteProcess 实例
        self.pid = pid

class ProcessExited(Event):
    """进程退出"""
    NAME = 'launch.events.process.ProcessExited'
    def __init__(self, *, action, pid, returncode, ...):
        self.returncode = returncode  # 0 = 正常退出，非0 = 异常

class ProcessStdout(Event):
    """进程标准输出（逐行）"""
    
class ProcessStderr(Event):
    """进程标准错误（逐行）"""

# ★ 系统事件
class Shutdown(Event):
    """请求关闭 LaunchService（Ctrl+C 或显式调用）"""
    NAME = 'launch.events.Shutdown'
    def __init__(self, *, reason: str = 'unspecified', ...):
        self.reason = reason

class IncludeLaunchDescription(Event):
    """请求包含新的 LaunchDescription"""
```

---

## 三、OnProcessExit — 进程退出响应

```python
# launch/launch/event_handlers/on_process_exit.py

class OnProcessExit(EventHandler):
    """当某个进程退出时执行 Actions"""
    
    def __init__(
        self,
        *,
        target_action: Optional[Union[ExecuteProcess, Callable]] = None,
        on_exit: Optional[Union[SomeActionsType, Callable]] = None,
    ):
        def handle(event: ProcessExited, context: LaunchContext):
            # ★ 检查是否是目标进程
            if target_action is not None:
                if isinstance(target_action, ExecuteProcess):
                    if event.action is not target_action:
                        return None
                elif callable(target_action):
                    if not target_action(event.action):
                        return None
            
            # ★ 执行 on_exit actions
            if callable(on_exit):
                return on_exit(event, context)
            return on_exit
        
        super().__init__(matcher=lambda event: isinstance(event, ProcessExited),
                         handler=handle)

# 使用示例1：某节点崩溃时关闭整个 Launch
from launch.actions import Shutdown

Node(
    package='critical_node',
    executable='critical_exe',
    on_exit=[Shutdown(reason='Critical node exited')],
)

# 使用示例2：RegisterEventHandler 方式
RegisterEventHandler(
    OnProcessExit(
        target_action=my_node,
        on_exit=[
            LogInfo(msg='my_node exited, restarting...'),
            my_node,  # ★ 重新启动同一个节点
        ]
    )
)
```

---

## 四、OnProcessStart — 进程启动后触发

```python
# 使用场景：等某节点完全启动后再启动依赖节点
# （注意：ProcessStarted ≠ 节点完全初始化完成，只是进程 fork 成功）

RegisterEventHandler(
    OnProcessStart(
        target_action=ros_bridge,
        on_start=[
            LogInfo(msg='ros_bridge started, launching web client...'),
            Node(
                package='web_client',
                executable='web_client_node',
            )
        ]
    )
)
```

---

## 五、OnShutdown — 优雅关闭处理

```python
# launch/launch/event_handlers/on_shutdown.py

class OnShutdown(EventHandler):
    def __init__(self, *, on_shutdown: Optional[SomeActionsType] = None):
        super().__init__(
            matcher=lambda event: isinstance(event, Shutdown),
            handler=lambda event, context: on_shutdown
        )

# 使用示例：关闭时执行清理
from launch.actions import ExecuteProcess, LogInfo

RegisterEventHandler(
    OnShutdown(on_shutdown=[
        LogInfo(msg='Shutting down, cleaning up...'),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/save_map', '...'],
            output='screen'
        )
    ])
)
```

---

## 六、TimerAction — 延迟执行

```python
# launch/launch/actions/timer_action.py

class TimerAction(Action):
    """在指定延迟后执行 Actions"""
    
    def __init__(
        self,
        *,
        period: Union[float, SomeSubstitutionsType],
        actions: List[LaunchDescriptionEntity],
        cancel_on_shutdown: bool = True
    ):
        self.__period = period
        self.__actions = actions
    
    def execute(self, context: LaunchContext):
        # ★ 使用 asyncio.sleep 延迟
        async def delayed_execute():
            await asyncio.sleep(self.__period)
            for action in self.__actions:
                context.emit_event_sync(IncludeLaunchDescription(
                    LaunchDescription([action])))
        
        context.asyncio_loop.create_task(delayed_execute())

# 使用示例：等待 3 秒后启动依赖节点
TimerAction(
    period=3.0,
    actions=[
        Node(package='rviz2', executable='rviz2', ...),
    ]
)
```

---

## 七、LogInfo — Launch 日志输出

```python
# launch/launch/actions/log_info.py

class LogInfo(Action):
    def __init__(self, *, msg: SomeSubstitutionsType):
        self.__msg = msg
    
    def execute(self, context: LaunchContext):
        # ★ 在 Launch 启动时打印消息（可含 Substitution）
        msg = perform_substitutions(context, self.__msg)
        launch.logging.get_logger().info(msg)
        return []

# 使用场景：调试 Substitution 值
LogInfo(msg=['Starting with robot_model=', LaunchConfiguration('robot_model')]),
```

---

## 八、RegisterEventHandler vs on_exit 参数

```python
# 两种方式等价：

# ★ 方式1：Node 的 on_exit 参数（更简洁）
Node(
    package='my_pkg',
    executable='my_exe',
    on_exit=Shutdown()
)

# ★ 方式2：RegisterEventHandler（更灵活）
my_node = Node(package='my_pkg', executable='my_exe')

RegisterEventHandler(
    OnProcessExit(
        target_action=my_node,
        on_exit=Shutdown()
    )
)

# 区别：方式2 可以在 LaunchDescription 中的任意位置注册
# 方式1 只能针对该节点本身
```

---

## 九、完整事件驱动示例

```python
def generate_launch_description():
    # 声明关键节点
    map_server = Node(package='nav2_map_server', executable='map_server', ...)
    amcl = Node(package='nav2_amcl', executable='amcl', ...)
    bt_navigator = Node(package='nav2_bt_navigator', executable='bt_navigator', ...)
    
    return LaunchDescription([
        # 启动地图服务器
        map_server,
        
        # 地图服务器启动后，等 2 秒再启动 AMCL
        RegisterEventHandler(
            OnProcessStart(
                target_action=map_server,
                on_start=[
                    TimerAction(period=2.0, actions=[amcl])
                ]
            )
        ),
        
        # AMCL 启动后，启动导航器
        RegisterEventHandler(
            OnProcessStart(
                target_action=amcl,
                on_start=[bt_navigator]
            )
        ),
        
        # 任一关键节点退出，关闭整个系统
        RegisterEventHandler(
            OnProcessExit(
                target_action=map_server,
                on_exit=Shutdown(reason='map_server crashed')
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=amcl,
                on_exit=Shutdown(reason='amcl crashed')
            )
        ),
        
        # 关闭时打印消息
        RegisterEventHandler(
            OnShutdown(on_shutdown=[
                LogInfo(msg='Navigation system shutting down.')
            ])
        ),
    ])
```
