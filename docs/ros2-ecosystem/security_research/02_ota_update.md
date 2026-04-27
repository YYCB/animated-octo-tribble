# OTA 固件/软件更新

> 覆盖 ROS 2 容器化 OTA 蓝绿部署、嵌入式 Linux OSTree/SWUpdate A/B 分区、micro-ROS MCU OTA、cosign/TUF 签名验证，以及 Mender.io 机器人 OTA 平台。

---

## 一、ROS 2 容器化部署与 OTA

### 1.1 Docker 镜像版本管理策略

```
镜像版本管理：

Harbor/GHCR 镜像仓库
├── robot/chassis:1.2.3          ← 稳定版（生产）
├── robot/chassis:1.2.4-rc1      ← 候选版（测试）
├── robot/chassis:latest         ← ⚠️ 不在生产中使用 latest
└── robot/chassis:sha-abc1234    ← commit SHA 固定版本（最安全）
```

```yaml
# docker-compose.prod.yml — 生产环境固定版本
version: '3.9'
services:
  chassis_node:
    image: harbor.robot.local/robot/chassis@sha256:abc1234...  # digest 固定
    runtime: nvidia
    privileged: false
    cap_add: [SYS_NICE]          # 允许 SCHED_FIFO
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    environment:
      - ROS_DOMAIN_ID=42
      - ROS_SECURITY_ENABLE=true
      - ROS_SECURITY_KEYSTORE=/run/secrets/keystore
    secrets:
      - keystore
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "ros2", "topic", "list"]
      interval: 30s
      timeout: 10s
      retries: 3

secrets:
  keystore:
    file: /opt/robot/security/keystore.tar.gz
```

### 1.2 蓝绿部署（零停机更新）

```bash
#!/bin/bash
# /opt/robot/scripts/ota_update.sh
# 蓝绿部署：新版本启动成功后切流，旧版本保留回滚

set -euo pipefail

NEW_IMAGE="harbor.robot.local/robot/chassis:${VERSION}"
HEALTH_CHECK_URL="http://localhost:8080/health"
MAX_WAIT=60  # 健康检查最大等待时间（秒）

# Step 1: 拉取并验证新镜像
echo "[1/5] Pulling and verifying new image..."
docker pull "$NEW_IMAGE"

# cosign 签名验证（见下方）
cosign verify \
  --certificate-identity "ci@robot.local" \
  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \
  "$NEW_IMAGE"

# Step 2: 启动新版本（蓝）
echo "[2/5] Starting new version (blue)..."
docker run -d \
  --name chassis_node_blue \
  --network robot_net \
  "$NEW_IMAGE"

# Step 3: 健康检查
echo "[3/5] Waiting for health check..."
for i in $(seq 1 $MAX_WAIT); do
  if docker exec chassis_node_blue ros2 topic echo /diagnostics --once 2>/dev/null; then
    echo "Health check passed!"
    break
  fi
  sleep 1
  if [ $i -eq $MAX_WAIT ]; then
    echo "Health check FAILED! Rolling back..."
    docker stop chassis_node_blue && docker rm chassis_node_blue
    exit 1
  fi
done

# Step 4: 切流（停止旧版本，新版本接管）
echo "[4/5] Switching traffic..."
docker rename chassis_node chassis_node_green_backup
docker rename chassis_node_blue chassis_node
docker stop chassis_node_green_backup

# Step 5: 保留旧版本 48h（快速回滚窗口）
echo "[5/5] Update complete. Old version retained for 48h rollback."
echo "To rollback: ./rollback.sh"
```

---

## 二、OSTree / SWUpdate 嵌入式 Linux OTA

### 2.1 OSTree A/B 分区架构

OSTree 实现文件系统级的原子 A/B 更新，适合 Jetson/ARM SoM 平台：

```
OSTree A/B 分区布局：

┌───────────────────────────────────────────────────────┐
│  bootloader (UEFI/U-Boot)                             │
│  ├── /boot/loader.conf → 指向活动分区（A 或 B）          │
│  └── 包含 A/B 各自的内核 + initrd                      │
├─────────────────────────────────┬─────────────────────┤
│  rootfs A（当前活动）             │  rootfs B（更新中）   │
│  /sysroot/ostree/deploy/robot/  │  （拉取新版本）        │
│  deploy/<hash_A>/               │                     │
│  ├── usr/ (只读挂载)             │                     │
│  ├── etc/ (可写覆盖层)           │                     │
│  └── var/ (持久数据)             │                     │
└─────────────────────────────────┴─────────────────────┘
```

```bash
# 服务端：发布新的 OSTree commit
ostree commit \
  --branch=robot/arm64/stable \
  --subject="v1.2.4: ROS2 Humble + chassis_protocol update" \
  /path/to/new_rootfs/

# 签名（GPG）
ostree gpg-sign robot/arm64/stable <GPG_KEY_ID>

# 客户端（机器人）：拉取并应用更新
ostree pull --depth=1 origin robot/arm64/stable
ostree admin upgrade

# 重启后自动使用新版本；旧版本保留（可手动 rollback）
systemctl reboot

# 回滚：
ostree admin rollback
systemctl reboot
```

### 2.2 SWUpdate 增量更新

SWUpdate 适合资源受限的 ARM 设备（内存 < 1GB），支持增量 delta 更新：

```json
// sw-description（SWUpdate manifest）
{
  "software": {
    "version": "1.2.4",
    "hardware-compatibility": ["robot-v2", "robot-v3"],
    "images": [
      {
        "filename": "rootfs.ext4.zst",
        "type": "raw",
        "device": "/dev/mmcblk0p2",
        "sha256": "abc123...",
        "compressed": "zstd",
        "installed-directly": false
      }
    ],
    "scripts": [
      {
        "filename": "post_install.sh",
        "type": "shellscript",
        "data": "#!/bin/bash\nros2 lifecycle set /chassis_node shutdown || true"
      }
    ],
    "signing": {
      "algorithm": "ed25519",
      "public_key": "/etc/swupdate/pubkey.pem"
    }
  }
}
```

---

## 三、micro-ROS MCU OTA

### 3.1 ESP32 OTA + micro-ROS 重连（Phase 8 对接）

```c
// esp32_ota_with_microros.c（ESP-IDF + micro-ROS）

#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>

// OTA 任务
void ota_task(void *pvParameter) {
    // 1. 停止 micro-ROS Agent 连接
    rcl_node_fini(&node);
    rcl_shutdown(&init_options);

    // 2. 验证固件签名（ED25519）
    esp_https_ota_config_t ota_config = {
        .http_config = {
            .url = OTA_SERVER_URL,
            .cert_pem = server_cert_pem,  // 服务端证书验证
        },
    };

    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &ota_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    // 3. 逐块下载并写入 OTA 分区
    while (true) {
        err = esp_https_ota_perform(ota_handle);
        if (err == ESP_ERR_HTTPS_OTA_IN_PROGRESS) continue;
        if (err == ESP_OK) break;
        ESP_LOGE(TAG, "OTA perform failed");
        esp_https_ota_abort(ota_handle);
        vTaskDelete(NULL);
        return;
    }

    // 4. 验证下载完整性（SHA256 校验）
    if (esp_https_ota_is_complete_data_received(ota_handle) != true) {
        ESP_LOGE(TAG, "Incomplete OTA data!");
        esp_https_ota_abort(ota_handle);
        vTaskDelete(NULL);
        return;
    }

    esp_err_t ota_finish_err = esp_https_ota_finish(ota_handle);
    if (ota_finish_err == ESP_OK) {
        ESP_LOGI(TAG, "OTA success! Rebooting...");
        esp_restart();  // 重启后自动启动新固件（A/B 切换）
    }
}

// 重启后 micro-ROS 重连
void app_main(void) {
    // ... 初始化
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, rcl_get_default_allocator());

    // 自动重连（带指数退避）
    uint32_t retry_delay_ms = 500;
    while (true) {
        rcl_ret_t ret = rclc_support_init_with_options(
            &support, 0, NULL, &init_options, &allocator
        );
        if (ret == RCL_RET_OK) {
            ESP_LOGI(TAG, "micro-ROS Agent connected");
            break;
        }
        ESP_LOGW(TAG, "Reconnect in %ums...", retry_delay_ms);
        vTaskDelay(retry_delay_ms / portTICK_PERIOD_MS);
        retry_delay_ms = MIN(retry_delay_ms * 2, 30000);  // 最大 30s
    }
}
```

---

## 四、签名验证

### 4.1 cosign 镜像签名

```bash
# CI/CD 中签名（GitHub Actions）
# .github/workflows/build_and_sign.yml

- name: Sign Docker image
  uses: sigstore/cosign-installer@v3
  
- name: Build and push
  run: |
    docker build -t harbor.robot.local/robot/chassis:${{ github.sha }} .
    docker push harbor.robot.local/robot/chassis:${{ github.sha }}

- name: Sign with cosign (keyless OIDC)
  run: |
    cosign sign \
      --yes \
      harbor.robot.local/robot/chassis:${{ github.sha }}

# 机器人端验证
- name: Verify on robot
  run: |
    cosign verify \
      --certificate-identity "https://github.com/YYCB/animated-octo-tribble/.github/workflows/build.yml@refs/heads/main" \
      --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \
      harbor.robot.local/robot/chassis:${{ github.sha }}
```

### 4.2 TUF（The Update Framework）元数据

TUF 提供带有多签名和防回滚攻击保护的软件更新元数据框架：

```
TUF 角色结构：

Root（根）         → 信任所有其他角色（离线签名，高安全性）
    │
    ├── Targets   → 列出所有可更新的目标文件及其 hash
    │    └── chassis-v1.2.4.tar.gz → sha256: abc123...
    │
    ├── Snapshot  → 列出所有 Targets 元数据版本（防版本降级）
    └── Timestamp → 当前有效时间戳（防回放攻击，短过期：1天）
```

```python
# Python TUF client（机器人端验证）
from tuf.ngclient import Updater

def verify_and_download_update(target_path: str) -> bytes:
    updater = Updater(
        metadata_dir="/etc/tuf/metadata/",
        metadata_base_url="https://updates.robot.local/",
        target_base_url="https://updates.robot.local/targets/",
        target_dir="/opt/robot/updates/",
    )
    updater.refresh()  # 更新 TUF 元数据（验证签名链）

    # 获取目标信息（包含 hash 和大小）
    target = updater.get_targetinfo(target_path)
    if target is None:
        raise ValueError(f"Target {target_path} not found in TUF metadata")

    # 下载并验证完整性
    path = updater.find_cached_target(target)
    if path is None:
        path = updater.download_target(target)  # 下载+SHA256验证

    return path.read_bytes()
```

---

## 五、Mender.io 机器人 OTA 平台

### 5.1 架构概述

```
Mender Server（云端）
    │  HTTPS（mutual TLS）
    ▼
Mender Client（机器人端，systemd daemon）
    ├── 定期轮询新 artifact（可配置间隔：默认 1800s）
    ├── 下载 artifact（增量 delta 支持）
    ├── 验证签名（artifact signing key）
    └── 触发 SWUpdate 或 OSTree 安装
```

### 5.2 Artifact 格式

```bash
# 创建 Mender artifact（包含 ROS 2 应用更新）
mender-artifact write rootfs-image \
  --artifact-name "chassis-protocol-v1.2.4" \
  --device-type "jetson-orin-nx" \
  --payload rootfs_v1.2.4.ext4 \
  --output chassis-protocol-v1.2.4.mender \
  --signing-key /path/to/artifact-signing-key.pem

# Delta 更新（节省带宽）
mender-artifact delta \
  --original chassis-protocol-v1.2.3.mender \
  --updated chassis-protocol-v1.2.4.mender \
  --output chassis-protocol-v1.2.3-to-v1.2.4-delta.mender
```

---

## 六、OTA 安全流程总结

```
完整 OTA 更新流程（安全保证）：

1. 更新前备份
   ├── OSTree：自动保留旧版本 commit（A 分区）
   └── Docker：旧容器重命名保留 48h

2. 签名验证
   ├── cosign / TUF 验证镜像/artifact 完整性
   └── ED25519 验证 MCU 固件签名

3. 原子部署
   ├── OSTree：文件系统级原子切换（重启后生效）
   ├── Docker：蓝绿切换（零停机）
   └── ESP32：A/B 分区原子切换（重启后生效）

4. 健康检查
   ├── ROS 2 话题可用性检查
   ├── 关键节点 lifecycle 状态检查
   └── 硬件自检（can ping MCU？）

5. 自动回滚触发条件
   ├── 健康检查超时（60s）
   ├── 关键节点 crashed（watchdog 检测）
   └── 用户手动触发
```
