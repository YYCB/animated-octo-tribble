# Launch 系统（一）：架构与执行流程

## 一、Launch 系统层次结构

```
launch（纯 Python 包，无 ROS 依赖）：
  ├── LaunchService       ← 主执行引擎（asyncio event loop）
  ├── LaunchContext       ← 运行时状态（变量、substitution 上下文）
  ├── LaunchDescription   ← 实体列表的容器
  ├── LaunchDescriptionEntity ← 所有实体的基类
  ├── actions/            ← ExecuteProcess, IncludeLaunch, GroupAction...
  ├── events/             ← Shutdown, ProcessStarted, ProcessExited...
  ├── event_handlers/     ← OnShutdown, OnProcessStart, OnProcessExit...
  └── substitutions/      ← LaunchConfiguration, EnvironmentVariable...

launch_ros（ROS2 专用扩展包）：
  ├── actions/
  │   ├── Node            ← 启动 ROS2 节点（最常用）
  │   ├── ComposableNodeContainer ← 启动 ComponentContainer
  │   ├── LoadComposableNodes  ← 动态加载组件
  │   ├── SetParameter    ← 设置全局参数
  │   └── PushRosNamespace ← 命名空间管理
  └── substitutions/
      ├── FindPackageShare ← 查找包的 share 目录
      └── ExecutableInPackage ← 找包的可执行文件
```

---

## 二、LaunchDescription 数据结构

```python
# launch/launch/launch_description.py

class LaunchDescription(LaunchDescriptionEntity):
    def __init__(
        self,
        initial_entities: Iterable[LaunchDescriptionEntity] = None,
        *,
        deprecated_reason: Optional[str] = None
    ):
        self.__entities = list(initial_entities or [])
        self.__deprecated_reason = deprecated_reason
    
    def get_launch_description_entities(self) -> Iterable[LaunchDescriptionEntity]:
        return self.__entities
    
    def visit(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """LaunchService 调用此方法展开实体"""
        if self.__deprecated_reason:
            launch.logging.get_logger().warning(
                f'This launch description is deprecated: {self.__deprecated_reason}')
        return self.__entities
```

---

## 三、LaunchService — 核心执行引擎

```python
# launch/launch/launch_service.py

class LaunchService:
    def __init__(
        self,
        *,
        argv: Iterable[str] = None,
        noninteractive: bool = False,
        debug: bool = False
    ):
        self.__context = LaunchContext(argv=argv)
        self.__loop = asyncio.new_event_loop()  # ★ 独立 asyncio 事件循环
        self.__event_queue = asyncio.Queue(loop=self.__loop)
        self.__process_manager = None
    
    def run(self, *, shutdown_when_idle: bool = True) -> int:
        """同步运行（阻塞直到完成）"""
        return self.__loop.run_until_complete(
            self.__run_impl(shutdown_when_idle=shutdown_when_idle)
        )
    
    async def __run_impl(self, shutdown_when_idle: bool) -> int:
        # ★ Step 1: 初始化进程管理器
        self.__process_manager = ProcessManager(
            self.__context, self.__event_queue)
        
        # ★ Step 2: 将初始 LaunchDescription 入队
        await self.__event_queue.put(
            IncludeLaunchDescription(self.__launch_description))
        
        # ★ Step 3: 主事件循环
        while True:
            try:
                # 等待下一个实体或事件
                entity = await asyncio.wait_for(
                    self.__event_queue.get(),
                    timeout=1.0
                )
            except asyncio.TimeoutError:
                if shutdown_when_idle and self.__is_idle():
                    break
                continue
            
            # ★ Step 4: 执行实体（visit）
            if isinstance(entity, LaunchDescriptionEntity):
                new_entities = entity.visit(self.__context)
                if new_entities:
                    for e in new_entities:
                        await self.__event_queue.put(e)
            
            # ★ Step 5: 处理事件（Event）
            elif isinstance(entity, Event):
                await self.__process_event(entity)
        
        return self.__context.locals.get('__return_code', 0)
```

---

## 四、LaunchContext — 运行时状态

```python
# launch/launch/launch_context.py

class LaunchContext:
    def __init__(self, *, argv: Iterable[str] = None):
        self.__argv = list(argv or [])
        
        # ★ 局部变量存储（LaunchConfiguration 值存在这里）
        self.__locals_stack: List[dict] = [{}]
        
        # ★ 事件处理器注册表
        self.__event_handlers: List[EventHandler] = []
        
        # ★ 环境变量覆盖
        self.__environment: dict = {}
    
    @property
    def locals(self) -> dict:
        """当前作用域的局部变量"""
        return self.__locals_stack[-1]
    
    def extend_locals(self, new_locals: dict):
        """在当前作用域添加/更新变量（LaunchConfiguration 使用）"""
        self.__locals_stack[-1].update(new_locals)
    
    def push_locals(self):
        """进入新作用域（GroupAction 使用，避免变量污染）"""
        self.__locals_stack.append(dict(self.__locals_stack[-1]))
    
    def pop_locals(self):
        """退出作用域（GroupAction 结束时）"""
        self.__locals_stack.pop()
    
    def perform_substitution(self, substitution) -> str:
        """解析 Substitution 对象为字符串"""
        if isinstance(substitution, str):
            return substitution
        return substitution.perform(self)
    
    def register_event_handler(self, event_handler: EventHandler):
        self.__event_handlers.append(event_handler)
    
    def get_event_handlers_for_event(self, event: Event):
        """找到所有匹配此事件类型的处理器"""
        return [h for h in self.__event_handlers if h.matches(event)]
```

---

## 五、generate_launch_description() — 标准入口

```python
# 标准 .launch.py 文件结构

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ★ Step 1: 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='ur5e',
        choices=['ur3e', 'ur5e', 'ur10e'],
        description='Robot model type'
    )
    
    # ★ Step 2: 读取参数值（延迟求值）
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')
    
    # ★ Step 3: 构建路径
    pkg_share = FindPackageShare('my_robot_description')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', robot_model, 'robot.urdf.xacro'])
    
    # ★ Step 4: 定义节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': urdf_path,
        }]
    )
    
    # ★ Step 5: 条件性包含
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py'
        ]),
        condition=IfCondition(use_sim_time),
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_model_arg,
        robot_state_publisher,
        gazebo_launch,
    ])
```

---

## 六、Launch 文件的 3 种格式

```
1. Python（.launch.py）：最灵活，支持条件/循环/动态生成
   → 推荐使用

2. XML（.launch.xml）：类似 ROS1，声明式，不支持逻辑
   <launch>
     <arg name="use_sim_time" default="false"/>
     <node pkg="..." exec="..." name="...">
       <param name="use_sim_time" value="$(var use_sim_time)"/>
     </node>
   </launch>

3. YAML（.launch.yaml）：更简洁的声明式格式（较少使用）
   launch:
     - arg:
         name: use_sim_time
         default: 'false'
     - node:
         pkg: ...
         exec: ...

XML/YAML 在底层也会被解析为相同的 LaunchDescription 对象
解析器：launch/parsers/
```

---

## 七、ros2 launch 命令到 LaunchService 的调用链

```python
# ros2launch/ros2launch/api/api.py

def launch_a_launch_file(
    launch_file_path: str,
    launch_arguments: List[str],
    debug: bool = False
) -> int:
    # 1. 动态 import .launch.py 文件
    spec = importlib.util.spec_from_file_location(
        '__main__', launch_file_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    
    # 2. 调用 generate_launch_description()
    ld = mod.generate_launch_description()
    
    # 3. 创建 LaunchService
    ls = LaunchService(argv=launch_arguments, debug=debug)
    ls.include_launch_description(ld)
    
    # 4. ★ 运行（阻塞）
    return ls.run()
```
