# 安全与 OTA 与本仓库的对接

> 将 Phase 11 安全技术栈与本仓库 `chassis_protocol` 和整体部署方案进行具体对接，给出可落地的配置步骤和选型决策矩阵。

---

## 一、chassis_protocol sros2 配置

### 1.1 ChassisNode 发布/订阅话题安全清单

根据 `chassis_protocol/CMakeLists.txt` 和 `ros2_adapter/chassis_node.cpp` 分析，ChassisNode 的话题访问权限应配置如下：

```xml
<!-- chassis_node_policy.xml — ChassisNode Topic ACL -->
<allow_rule>
  <domains><id>0</id></domains>
  <!-- ChassisNode 发布 -->
  <publish>
    <topics>
      <topic>/odom</topic>                   <!-- 里程计 -->
      <topic>/joint_states</topic>           <!-- 关节状态 -->
      <topic>/diagnostics</topic>            <!-- 诊断信息 -->
      <topic>/tf</topic>                     <!-- 坐标变换 -->
      <topic>/rosout</topic>                 <!-- 日志 -->
    </topics>
  </publish>
  <!-- ChassisNode 订阅 -->
  <subscribe>
    <topics>
      <topic>/cmd_vel</topic>                <!-- 速度指令 -->
      <topic>/joint_commands</topic>         <!-- 关节指令 -->
      <topic>/parameter_events</topic>       <!-- 参数服务 -->
      <topic>/rosout</topic>
    </topics>
  </subscribe>
</allow_rule>
<deny_rule>
  <domains><id>0</id></domains>
  <publish><topics><topic>*</topic></topics></publish>
  <subscribe><topics><topic>*</topic></topics></subscribe>
</deny_rule>
<default>DENY</default>
```

### 1.2 完整证书部署脚本

```bash
#!/bin/bash
# scripts/setup_sros2.sh — chassis_protocol sros2 一键配置

set -euo pipefail

KEYSTORE="/opt/robot/security/keystore"
POLICY_DIR="/opt/robot/security/policies"

# 环境检查
ros2 security --help > /dev/null 2>&1 || {
  echo "ERROR: sros2 not available. Install: sudo apt install ros-humble-rmw-fastrtps-cpp"
  exit 1
}

# 1. 创建 keystore
echo "[1/6] Creating keystore..."
ros2 security create_keystore "$KEYSTORE"

# 2. 为所有节点创建证书
echo "[2/6] Creating node certificates..."
NODES=(
  /chassis_node
  /controller_manager
  /robot_state_publisher
  /nav2_stack
  /moveit_move_group
)
for node in "${NODES[@]}"; do
  ros2 security create_key "$KEYSTORE" "$node"
  echo "  Created: $node"
done

# 3. 应用权限策略
echo "[3/6] Applying permissions..."
ros2 security create_permission "$KEYSTORE" /chassis_node \
  "$POLICY_DIR/chassis_node_policy.xml"

# 4. 验证所有 profiles
echo "[4/6] Validating profiles..."
for node in "${NODES[@]}"; do
  ros2 security check_profile "$KEYSTORE" "$node"
done

# 5. 设置文件权限（防止未授权读取私钥）
echo "[5/6] Setting file permissions..."
chmod -R 700 "$KEYSTORE/private"
chmod -R 700 "$KEYSTORE/enclaves"
chown -R robot:robot "$KEYSTORE"

echo "[6/6] sros2 setup complete!"
echo "Set these environment variables before launching:"
echo "  export ROS_SECURITY_ENABLE=true"
echo "  export ROS_SECURITY_STRATEGY=Enforce"
echo "  export ROS_SECURITY_KEYSTORE=$KEYSTORE"
```

---

## 二、整体安全架构

### 2.1 认证/加密/审计的层次设计

```
具身智能机器人整体安全架构（纵深防御）：

┌─────────────────────────────────────────────────────────────────────┐
│  L1 物理安全层                                                        │
│  ├── 硬件加密存储（TPM 2.0 / Secure Enclave）→ 保护 sros2 私钥        │
│  ├── 物理访问控制（机箱锁）                                            │
│  └── 磁盘加密（dm-crypt/LUKS）→ 保护 rosbag2 录制数据                 │
├─────────────────────────────────────────────────────────────────────┤
│  L2 系统安全层                                                        │
│  ├── AppArmor / SELinux → ROS 2 节点文件系统隔离                      │
│  ├── seccomp → 限制节点系统调用                                        │
│  ├── 只读根文件系统（OSTree）                                          │
│  └── 最小权限用户（非 root 运行 chassis_node）                         │
├─────────────────────────────────────────────────────────────────────┤
│  L3 网络/通信安全层（sros2 / DDS-Security）                            │
│  ├── 认证：X.509 证书，每节点独立证书                                  │
│  ├── 加密：AES-256-GCM（控制话题）                                     │
│  ├── 访问控制：permissions.xml Topic ACL                              │
│  └── 审计：安全事件日志（→ SIEM）                                      │
├─────────────────────────────────────────────────────────────────────┤
│  L4 应用安全层                                                        │
│  ├── 安全停机（E-Stop）：认证后才能执行                                 │
│  ├── 速度限制：chassis_protocol 硬件限速（不依赖软件）                   │
│  └── 输入验证：cmd_vel 范围检查（防注入异常值）                          │
├─────────────────────────────────────────────────────────────────────┤
│  L5 更新安全层（OTA）                                                  │
│  ├── 镜像签名验证（cosign + TUF）                                      │
│  ├── A/B 分区原子更新（OSTree / SWUpdate）                             │
│  ├── MCU 固件 OTA 签名（ED25519）                                      │
│  └── 健康检查 + 自动回滚                                               │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 安全事件响应流程

```
检测到安全事件（DDS-Security 日志 / 系统审计）
    │
    ▼
事件分类：
  P1（高）：恶意 cmd_vel 注入 → 立即 E-Stop
  P2（中）：未认证节点尝试连接 → 日志 + 告警
  P3（低）：权限文件过期 → 调度证书更新
    │
    ▼
P1 响应（< 10ms）：
  ├── hardware E-Stop（GPIO 或安全总线）
  └── 独立于 ROS 2 软件栈

P2/P3 响应（异步）：
  ├── 发送 Slack/PagerDuty 告警
  ├── 记录到 SIEM（ELK Stack）
  └── 调度 OTA 修复（如需）
```

---

## 三、OTA 部署流程（针对本仓库）

### 3.1 Docker 镜像 OTA 方案

```
仓库发布流程：

GitHub Repository
    │
    │ git push tag v1.2.4
    ▼
GitHub Actions CI
  ├── colcon build（ROS 2 Humble）
  ├── colcon test（unit tests）
  ├── docker build
  ├── docker push → GHCR
  └── cosign sign（keyless OIDC）
    │
    ▼
Mender Server（或自定义 OTA 服务）
  ├── 新镜像 artifact 可用通知
  └── 审批 + 分级推送（10% → 50% → 100%）
    │
    ▼
机器人 OTA Agent
  ├── 拉取 artifact
  ├── cosign verify
  ├── 蓝绿部署（chassis_node 无停机更新）
  └── 健康检查 → 确认 or 回滚
```

### 3.2 GitHub Actions CI/CD 集成

```yaml
# .github/workflows/robot_release.yml
name: Robot OTA Release

on:
  push:
    tags: ['v*']

jobs:
  build-and-release:
    runs-on: ubuntu-22.04
    permissions:
      contents: read
      packages: write
      id-token: write  # cosign keyless signing

    steps:
      - uses: actions/checkout@v4

      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: humble

      - name: Build
        run: |
          cd $GITHUB_WORKSPACE
          colcon build --packages-select chassis_protocol

      - name: Test
        run: colcon test --packages-select chassis_protocol

      - name: Build Docker image
        run: |
          docker build \
            --build-arg ROS_DISTRO=humble \
            -t ghcr.io/${{ github.repository }}/chassis:${{ github.ref_name }} \
            -f docker/Dockerfile.chassis .

      - name: Push to GHCR
        run: |
          echo "${{ secrets.GITHUB_TOKEN }}" | docker login ghcr.io -u $ --password-stdin
          docker push ghcr.io/${{ github.repository }}/chassis:${{ github.ref_name }}

      - name: Sign with cosign
        uses: sigstore/cosign-installer@v3
      - run: |
          cosign sign --yes \
            ghcr.io/${{ github.repository }}/chassis:${{ github.ref_name }}
```

---

## 四、选型决策矩阵

| 场景 | 推荐方案 | 优先级 | 实施复杂度 | 备注 |
|------|---------|-------|---------|------|
| **ROS 2 话题加密（控制指令）** | sros2 AES-256（控制话题） | ⭐⭐⭐⭐⭐ | 中 | 必须，防恶意指令注入 |
| **ROS 2 话题加密（图像）** | 不加密 + VPN 网络隔离 | ⭐⭐⭐ | 低 | 加密开销高，用网络隔离替代 |
| **节点访问控制 ACL** | sros2 permissions.xml | ⭐⭐⭐⭐⭐ | 中 | 最小权限原则 |
| **Docker 镜像完整性** | cosign（keyless OIDC） | ⭐⭐⭐⭐⭐ | 低 | CI 集成成本极低 |
| **嵌入式 Linux OTA** | OSTree（Jetson） | ⭐⭐⭐⭐ | 高 | 需要专项工程投入 |
| **MCU 固件 OTA** | ESP-IDF OTA + ED25519 | ⭐⭐⭐⭐⭐ | 中 | Phase 8 micro-ROS 场景必须 |
| **OTA 平台（商用）** | Mender.io | ⭐⭐⭐ | 低（托管） | 预算充足时推荐 |
| **OTA 平台（自建）** | GitHub Actions + 蓝绿部署 | ⭐⭐⭐⭐ | 中 | 开源首选 |
| **私钥保护** | TPM 2.0 | ⭐⭐⭐ | 高 | 高安全等级场景 |

**本仓库分阶段实施建议：**

| 阶段 | 工作内容 | 时间估算 |
|------|---------|---------|
| **P0（立即）** | cosign 镜像签名、GitHub Actions CI | 1 人天 |
| **P1（近期）** | sros2 控制话题加密（`/cmd_vel` + `/joint_commands`）| 3 人天 |
| **P2（中期）** | sros2 全话题 ACL、permissions.xml 维护工具 | 5 人天 |
| **P3（长期）** | OSTree A/B + Mender OTA 平台集成 | 2 人周 |
