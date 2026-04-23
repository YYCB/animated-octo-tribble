# Launch 系统（二）：核心 Actions 详解

## 一、launch_ros.actions.Node — 最重要的 Action

```python
# launch_ros/launch_ros/actions/node.py

class Node(ExecuteProcess):
    """启动一个 ROS2 节点"""
    
    def __init__(
        self,
        *,
        executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        exec_name: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        **kwargs
    ):
        self.__package = package
        self.__node_executable = executable
        self.__node_name = name
        self.__node_namespace = namespace
        self.__parameters = parameters or []
        self.__remappings = remappings or []
        
        # ★ 继承 ExecuteProcess，Node 最终是一个子进程
        super().__init__(cmd=[], **kwargs)
    
    def execute(self, context: LaunchContext):
        """构建实际命令行"""
        
        # ★ Step 1: 解析包名 → 找可执行文件路径
        if self.__package:
            pkg_name = perform_substitutions(context, self.__package)
            # 使用 ament_index 找到可执行文件
            exec_path = get_executable_path(pkg_name, 
                          perform_substitutions(context, self.__node_executable))
        
        # ★ Step 2: 构建 ROS 参数（--ros-args）
        ros_args = []
        
        # 节点名重映射
        if self.__node_name:
            name = perform_substitutions(context, self.__node_name)
            ros_args += ['--ros-args', '-r', f'__node:={name}']
        
        # 命名空间
        if self.__node_namespace:
            ns = perform_substitutions(context, self.__node_namespace)
            ros_args += ['-r', f'__ns:={ns}']
        
        # ★ 参数文件/内联参数
        for param in self.__parameters:
            if isinstance(param, dict):
                # 内联字典 → 写入临时 YAML 文件
                param_file = self._create_param_file(context, param)
                ros_args += ['--params-file', param_file]
            elif isinstance(param, (str, Substitution)):
                # YAML 文件路径
                ros_args += ['--params-file', 
                             perform_substitutions(context, param)]
        
        # ★ 重映射规则
        for src, dst in self.__remappings:
            src_str = perform_substitutions(context, src)
            dst_str = perform_substitutions(context, dst)
            ros_args += ['-r', f'{src_str}:={dst_str}']
        
        # ★ Step 3: 组合最终命令
        cmd = [exec_path] + arguments + ['--ros-args'] + ros_args
        
        # 传给 ExecuteProcess
        self.cmd = cmd
        return super().execute(context)
    
    def _create_param_file(self, context, param_dict):
        """将字典写入临时 YAML 文件（供 --params-file 使用）"""
        import tempfile, yaml
        
        node_name = perform_substitutions(context, self.__node_name) if self.__node_name else '*'
        
        yaml_content = {
            node_name: {'ros__parameters': {}}
        }
        for k, v in param_dict.items():
            # ★ 解析 Substitution 值
            resolved_key = perform_substitutions(context, k)
            resolved_val = perform_substitutions(context, v) if hasattr(v, 'perform') else v
            yaml_content[node_name]['ros__parameters'][resolved_key] = resolved_val
        
        f = tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False)
        yaml.dump(yaml_content, f)
        f.close()
        return f.name
```

---

## 二、ExecuteProcess — 通用进程启动

```python
# launch/launch/actions/execute_process.py

class ExecuteProcess(Action):
    def __init__(
        self,
        *,
        cmd: SomeSubstitutionsType,
        cwd: Optional[SomeSubstitutionsType] = None,
        env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        additional_env: Optional[Dict] = None,
        shell: bool = False,
        sigterm_timeout: SomeSubstitutionsType = '5',
        sigkill_timeout: SomeSubstitutionsType = '5',
        on_exit: Optional[Union[SomeActionsType, Callable]] = None,
        output: SomeSubstitutionsType = 'log',  # 'log' | 'screen' | 'both'
        **kwargs
    ):
        self.__cmd = cmd
        self.__output = output
        self.__on_exit_actions = on_exit
    
    def execute(self, context: LaunchContext):
        # ★ 解析命令（所有 Substitution 在此时求值）
        cmd = [perform_substitutions(context, c) for c in self.__cmd]
        
        # ★ 通过 asyncio subprocess 启动进程
        async def __execute_process_coroutine():
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                cwd=cwd,
                env=env,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            
            # ★ 将进程添加到进程管理器
            context.emit_event_sync(
                ProcessStarted(action=self, pid=proc.pid, cmd=cmd))
            
            # 等待退出
            returncode = await proc.wait()
            
            # ★ 发出进程退出事件（触发 OnProcessExit 处理器）
            context.emit_event_sync(
                ProcessExited(action=self, pid=proc.pid, returncode=returncode))
        
        context.asyncio_loop.create_task(__execute_process_coroutine())
```

---

## 三、IncludeLaunchDescription — 嵌套 Launch

```python
# launch/launch/actions/include_launch_description.py

class IncludeLaunchDescription(Action):
    def __init__(
        self,
        launch_description_source: LaunchDescriptionSource,
        *,
        launch_arguments: Iterable[Tuple] = None,
        condition: Optional[Condition] = None
    ):
        self.__launch_description_source = launch_description_source
        self.__launch_arguments = launch_arguments or []
    
    def execute(self, context: LaunchContext):
        # ★ 加载子 Launch 文件
        ld = self.__launch_description_source.get_launch_description(context)
        
        # ★ 将父 Launch 传入的参数注入子 context
        # （子 Launch 的 DeclareLaunchArgument 默认值会被覆盖）
        for name, value in self.__launch_arguments:
            context.extend_locals({
                perform_substitutions(context, name): 
                perform_substitutions(context, value)
            })
        
        # ★ 返回子 LaunchDescription 的实体列表
        return ld.visit(context)

# 使用示例：
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare('nav2_bringup'), '/launch/navigation_launch.py'
    ]),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'map': map_yaml_file,
    }.items()
)
```

---

## 四、GroupAction — 作用域隔离

```python
# launch_ros/launch_ros/actions/group_action.py

class GroupAction(Action):
    """将一组 Action 放在独立的命名空间/作用域中"""
    
    def __init__(
        self,
        actions: List[LaunchDescriptionEntity],
        *,
        scoped: bool = True,           # 是否隔离 LaunchConfiguration 变量
        forwarding: bool = True,       # 是否传递父作用域变量
        launch_configurations: dict = None,  # 额外配置
        condition: Optional[Condition] = None
    ):
        self.__actions = actions
        self.__scoped = scoped
    
    def execute(self, context: LaunchContext):
        # ★ 进入新作用域（push）
        if self.__scoped:
            context.push_locals()
        
        # 应用额外配置
        if self.__launch_configurations:
            context.extend_locals(self.__launch_configurations)
        
        # ★ 返回内部实体
        result_actions = list(self.__actions)
        
        # 注册退出时 pop 作用域
        if self.__scoped:
            result_actions.append(PopLaunch(context))
        
        return result_actions

# 使用示例（给一组节点加统一命名空间）：
GroupAction([
    PushRosNamespace('robot1'),
    Node(package='nav2_amcl', executable='amcl', ...),
    Node(package='nav2_bt_navigator', executable='bt_navigator', ...),
])
# 结果：节点名为 /robot1/amcl, /robot1/bt_navigator
```

---

## 五、DeclareLaunchArgument — 声明参数

```python
# launch/launch/actions/declare_launch_argument.py

class DeclareLaunchArgument(Action):
    def __init__(
        self,
        name: str,
        *,
        default_value: Optional[SomeSubstitutionsType] = None,
        description: str = '',
        choices: Optional[Iterable[str]] = None
    ):
        self.__name = name
        self.__default_value = default_value
        self.__choices = choices
    
    def execute(self, context: LaunchContext):
        # ★ 检查是否已从命令行传入
        if self.__name not in context.locals:
            if self.__default_value is None:
                raise RuntimeError(
                    f"Required launch argument '{self.__name}' not provided")
            # 使用默认值
            default = perform_substitutions(context, self.__default_value)
            context.extend_locals({self.__name: default})
        
        # ★ 如果有 choices，验证值是否合法
        if self.__choices:
            current_val = context.locals[self.__name]
            if current_val not in self.__choices:
                raise RuntimeError(
                    f"Invalid value '{current_val}' for argument '{self.__name}'. "
                    f"Valid choices: {self.__choices}")
        
        return []

# 命令行传入参数：
# ros2 launch my_pkg my.launch.py use_sim_time:=true robot_model:=ur10e
```

---

## 六、ComposableNodeContainer — 组件容器

```python
# launch_ros/launch_ros/actions/composable_node_container.py

class ComposableNodeContainer(Node):
    """启动一个组件容器（rclcpp_components::ComponentManager）"""
    
    def __init__(
        self,
        *,
        name: SomeSubstitutionsType,
        namespace: SomeSubstitutionsType,
        package: str = 'rclcpp_components',
        executable: str = 'component_container',
        composable_node_descriptions: List[ComposableNode] = None,
        **kwargs
    ):
        self.__composable_nodes = composable_node_descriptions or []
        super().__init__(package=package, executable=executable, **kwargs)
    
    def execute(self, context):
        # 先启动容器进程
        result = super().execute(context)
        
        # ★ 然后通过 LoadComposableNodes 加载组件
        if self.__composable_nodes:
            load_action = LoadComposableNodes(
                composable_node_descriptions=self.__composable_nodes,
                target_container=self.node_name
            )
            return (result or []) + [load_action]

# 使用示例：
ComposableNodeContainer(
    name='image_pipeline',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            remappings=[('image', '/camera/image_raw')],
        ),
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzNode',
            name='point_cloud_node',
        ),
    ],
)
```

---

## 七、OpaqueFunction — 动态生成 Action

```python
# 当需要在运行时（context 已知后）动态生成 Actions 时使用

from launch.actions import OpaqueFunction

def setup_nodes(context, *args, **kwargs):
    """在 context 已知后动态生成 Action 列表"""
    
    # ★ 此时可以 perform Substitution（读取 LaunchConfiguration）
    robot_model = LaunchConfiguration('robot_model').perform(context)
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    
    nodes = []
    for i in range(num_robots):
        nodes.append(Node(
            package='my_robot',
            executable='robot_node',
            namespace=f'robot{i}',
            parameters=[{'model': robot_model, 'id': i}]
        ))
    
    return nodes

# 使用：
LaunchDescription([
    DeclareLaunchArgument('robot_model', default_value='ur5e'),
    DeclareLaunchArgument('num_robots', default_value='1'),
    OpaqueFunction(function=setup_nodes),  # ★ 延迟执行
])
```
