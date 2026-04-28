# Launch 系统（四）：Substitution 替换系统

## 一、Substitution 基础概念

```python
# Substitution = 延迟求值的表达式
# 在 execute() 时才解析为字符串
# 核心方法：substitution.perform(context: LaunchContext) → str

# 为什么需要延迟求值？
# LaunchDescription 在 generate_launch_description() 中构建（Python import 时）
# 但 LaunchConfiguration 的值要等到 LaunchService.run() 时才确定
# Substitution 保证在正确的时机求值

# SomeSubstitutionsType 可以是：
#   str                          → 直接字符串
#   Substitution                 → 单个替换
#   List[Union[str, Substitution]] → 连接多个片段
```

---

## 二、LaunchConfiguration — 读取启动参数

```python
# launch/launch/substitutions/launch_configuration.py

class LaunchConfiguration(Substitution):
    """读取 DeclareLaunchArgument 声明的参数值"""
    
    def __init__(self, variable_name: str, *, default: Optional[str] = None):
        self.__variable_name = variable_name
        self.__default = default
    
    def perform(self, context: LaunchContext) -> str:
        # ★ 从 context.locals 中查找变量
        name = perform_substitutions(context, self.__variable_name)
        
        try:
            return context.locals[name]
        except KeyError:
            if self.__default is not None:
                return perform_substitutions(context, self.__default)
            raise SubstitutionFailure(
                f"LaunchConfiguration '{name}' was not set")

# 使用：
use_sim_time = LaunchConfiguration('use_sim_time')

# 在字符串中使用（列表拼接）：
Node(
    name=['robot_', LaunchConfiguration('robot_id')],  # → "robot_0"
    parameters=[{'log_prefix': ['[Robot ', LaunchConfiguration('robot_id'), ']']}]
)
```

---

## 三、FindPackageShare / PathJoinSubstitution

```python
# launch_ros/launch_ros/substitutions/find_package_share.py

class FindPackageShare(Substitution):
    """找到 ROS2 包的 share 目录（install/share/<pkg>）"""
    
    def __init__(self, package: SomeSubstitutionsType):
        self.__package = package
    
    def perform(self, context: LaunchContext) -> str:
        package_name = perform_substitutions(context, self.__package)
        # ★ 使用 ament_index_python 查找
        package_share_directory = get_package_share_directory(package_name)
        return package_share_directory
        # 例：/opt/ros/humble/share/nav2_bringup

# PathJoinSubstitution — 路径拼接（os.path.join 的 Substitution 版本）
class PathJoinSubstitution(Substitution):
    def __init__(self, substitutions: List[SomeSubstitutionsType]):
        self.__substitutions = substitutions
    
    def perform(self, context: LaunchContext) -> str:
        import os
        parts = [perform_substitutions(context, s) for s in self.__substitutions]
        return os.path.join(*parts)

# 使用：
config_file = PathJoinSubstitution([
    FindPackageShare('my_package'),
    'config',
    LaunchConfiguration('robot_model'),
    'params.yaml'
])
# → /opt/ros/humble/share/my_package/config/ur5e/params.yaml
```

---

## 四、PythonExpression — 动态计算

```python
# launch/launch/substitutions/python_expression.py

class PythonExpression(Substitution):
    """执行 Python 表达式，返回字符串结果"""
    
    def __init__(self, expression: SomeSubstitutionsType):
        self.__expression = expression
    
    def perform(self, context: LaunchContext) -> str:
        expression_str = perform_substitutions(context, self.__expression)
        
        # ★ 安全地 eval Python 表达式（有限的命名空间）
        result = eval(expression_str, {
            '__builtins__': {},  # 限制内置函数
            'True': True, 'False': False,
            'int': int, 'float': float, 'str': str,
            'len': len, 'max': max, 'min': min,
        })
        return str(result)

# 使用示例：
# 条件判断（字符串"true"/"false" → bool）
IfCondition(
    PythonExpression([
        "'", LaunchConfiguration('use_sim_time'), "' == 'true'"
    ])
)

# 数值计算：
Node(
    parameters=[{
        'update_rate': PythonExpression(
            [LaunchConfiguration('base_freq'), ' * 2']
        )
    }]
)
```

---

## 五、EnvironmentVariable — 读取环境变量

```python
# launch/launch/substitutions/environment_variable.py

class EnvironmentVariable(Substitution):
    def __init__(self, name: SomeSubstitutionsType, *, default: Optional[str] = None):
        self.__name = name
        self.__default = default
    
    def perform(self, context: LaunchContext) -> str:
        name = perform_substitutions(context, self.__name)
        value = os.environ.get(name, self.__default)
        if value is None:
            raise SubstitutionFailure(
                f"Environment variable '{name}' is not set")
        return value

# 使用：
Node(
    parameters=[{
        'robot_ip': EnvironmentVariable('ROBOT_IP', default='192.168.1.100'),
        'debug': EnvironmentVariable('ROS_DEBUG', default='false'),
    }]
)
```

---

## 六、条件（Condition）系统

```python
# launch/launch/conditions/

# ★ IfCondition — 字符串 "true"/"1" 时执行
class IfCondition(Condition):
    def __init__(self, predicate: SomeSubstitutionsType):
        self.__predicate = predicate
    
    def evaluate(self, context: LaunchContext) -> bool:
        result = perform_substitutions(context, self.__predicate)
        return result.lower() in ('true', '1', 'yes', 'on')

# ★ UnlessCondition — 取反
class UnlessCondition(Condition):
    def evaluate(self, context: LaunchContext) -> bool:
        result = perform_substitutions(context, self.__predicate)
        return result.lower() not in ('true', '1', 'yes', 'on')

# ★ LaunchConfigurationEquals — 精确匹配
class LaunchConfigurationEquals(Condition):
    def __init__(self, launch_configuration_name: str, expected_value: str):
        ...
    def evaluate(self, context: LaunchContext) -> bool:
        return context.locals.get(self.__name) == self.__expected_value

# ★ LaunchConfigurationNotEquals — 不等于
class LaunchConfigurationNotEquals(Condition): ...

# 使用示例：
Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    condition=IfCondition(LaunchConfiguration('use_sim_time'))
),
Node(
    package='hardware_driver',
    executable='driver_node',
    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
),

# ★ 组合条件（使用 PythonExpression）：
Node(
    condition=IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('mode'), "' == 'auto' and ",
            "'", LaunchConfiguration('use_sensor'), "' == 'true'"
        ])
    )
)
```

---

## 七、AnonName — 匿名唯一名称

```python
# launch/launch/substitutions/anon_name.py

class AnonName(Substitution):
    """生成匿名唯一名称（类似 ROS1 的 rospy.init_node anonymous=True）"""
    
    def perform(self, context: LaunchContext) -> str:
        # 追加随机数确保唯一
        import random, string
        suffix = ''.join(random.choices(string.digits, k=6))
        return f'anon_{suffix}'

# 使用：
Node(
    name=AnonName(),  # → 'anon_384729'
    ...
)
```

---

## 八、LocalSubstitution — 内部状态访问

```python
# launch/launch/substitutions/local_substitution.py

class LocalSubstitution(Substitution):
    """访问 LaunchContext.locals 中的任意变量"""
    
    def __init__(self, expression: str, description: Optional[str] = None):
        self.__expression = expression
    
    def perform(self, context: LaunchContext) -> str:
        # ★ 用 eval 访问 context.locals
        return str(eval(self.__expression, {}, context.locals))

# 内部使用（launch 框架内部）：
# OnProcessExit 中获取退出码：
LocalSubstitution('event.returncode', description='exit code of process')
```

---

## 九、Substitution 使用规范

```python
# ★ 字符串与 Substitution 混合（list 形式）：
Node(
    name=['robot_', LaunchConfiguration('robot_id'), '_controller'],
    # 等价于：f"robot_{robot_id}_controller"
)

# ★ 参数字典中的 Substitution：
Node(
    parameters=[{
        # value 可以是 Substitution：
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        
        # key 也可以是 Substitution（较少见）：
        LaunchConfiguration('param_name'): 'value',
        
        # 嵌套结构不直接支持，需要用 YAML 文件
    }]
)

# ★ 条件性参数文件：
Node(
    parameters=[
        PathJoinSubstitution([
            FindPackageShare('my_pkg'), 'config', 'base.yaml'
        ]),
        # 条件性额外参数文件
        PathJoinSubstitution([
            FindPackageShare('my_pkg'), 'config',
            [LaunchConfiguration('robot_model'), '_override.yaml']
        ]),
    ]
)

# ★ 重映射中的 Substitution：
Node(
    remappings=[
        ('input', ['/', LaunchConfiguration('namespace'), '/sensor_data']),
        ('output', '/processed_data'),
    ]
)
```
