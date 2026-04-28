# 08 RTPS 线协议 —— DDS 底层报文详解

> RTPS（Real-Time Publish-Subscribe Protocol）是 DDS 的网络线协议。**理解 RTPS 是理解 DDS 行为（可靠性、重传、分片、心跳）的根本**——QoS 的"神秘"现象多数是 RTPS 机制的直接体现。

---

## 8.1 RTPS 总体分层

```
┌────────────────────────────────────────────────┐
│ DDS API (DataWriter.write / DataReader.take)   │
├────────────────────────────────────────────────┤
│ RTPS Entity (Writer / Reader)                  │
├────────────────────────────────────────────────┤
│ RTPS Behavior (Stateless / Stateful)           │
├────────────────────────────────────────────────┤
│ RTPS Message  (Header + N × Submessage)        │
├────────────────────────────────────────────────┤
│ Transport (UDP / TCP / SHM)                    │
└────────────────────────────────────────────────┘
```

**一个 RTPS Message**：
- 16 字节 Header：`RTPS` 魔数、协议版本、vendor id、GuidPrefix；
- N 个 Submessage：变长，每个有 1 字节 id + flags + length + payload。

**GUID（Global Unique Identifier）**：每个 entity 的全局标识，=（12 字节 GuidPrefix +  4 字节 EntityId）。Participant/Writer/Reader 各自 GUID。

---

## 8.2 Submessage 类型一览

| Submessage | ID | 方向 | 用途 |
|------------|-----|------|------|
| `DATA` | 0x15 | W→R | 载荷（用户消息序列化后的 CDR 数据） |
| `DATA_FRAG` | 0x16 | W→R | 分片载荷（消息 > MTU 时） |
| `HEARTBEAT` | 0x07 | W→R | 通知 Reader 当前已发布的序号范围 |
| `HEARTBEAT_FRAG` | 0x13 | W→R | 分片级心跳 |
| `ACKNACK` | 0x06 | R→W | 确认已收 / 请求重传 |
| `NACK_FRAG` | 0x12 | R→W | 分片级 NACK |
| `GAP` | 0x08 | W→R | 显式告知 Reader 某些序号不会有数据（被过滤/丢弃） |
| `INFO_TS` | 0x09 | W→R | 时间戳 |
| `INFO_DST` | 0x0E | W→R | 目的 Participant（多播下指定） |
| `INFO_SRC` | 0x0C | W→R | 源信息 |
| `PAD` | 0x01 | — | 对齐填充 |

**最关键的五个**：DATA、HEARTBEAT、ACKNACK、GAP、DATA_FRAG。

---

## 8.3 DATA 子消息剖析

```
+--+--+--+--+--+--+--+--+
| DATA | flags | length  |
+--+--+--+--+--+--+--+--+
| extra flags | octets_to_next_header |
| readerEntityId (4)                  |
| writerEntityId (4)                  |
| writerSN (8)                        |  <-- 序列号
| inlineQos? (variable)               |
| serializedPayload (variable)        |
+--+--+--+--+--+--+--+--+
```

**关键字段**：
- `writerSN`（Sequence Number）：每个 writer 单调递增，Reader 用此排序 / 去重 / 检测丢失；
- `serializedPayload`：CDR 编码的用户消息；
- `inlineQos`：可选的运行时 QoS 覆盖（很少用）；
- `flags`：bit0=endianness, bit1=inlineQos present, bit2=data present, bit3=key present。

**注意**：DATA 本身**不包含 topic 名**——topic 的识别是通过 writerEntityId（发现期已建立 writer GUID ↔ topic 的映射）。

---

## 8.4 可靠性机制（RELIABLE QoS 的实际工作）

### 8.4.1 状态机

Writer 端为每个匹配 Reader 维护：
- `firstAvailableSN` / `lastAvailableSN`：当前 writer 缓存的序号范围；
- `acknowledgedSN`：对该 Reader 确认到哪一个序号；

Reader 端为每个匹配 Writer 维护：
- `receivedSN[]`：已收到哪些序号；
- `missingSN[]`：已知缺失的序号；
- `highestSN`：已收最大序号。

### 8.4.2 典型时序：正常传输

```
Writer                                   Reader
  │                                        │
  │── DATA (SN=100) ────────────────▶     │
  │── DATA (SN=101) ────────────────▶     │
  │── DATA (SN=102) ────────────────▶     │
  │                                        │
  │── HEARTBEAT (first=100, last=102) ─▶  │
  │                                        │
  │   ◀─ ACKNACK (bitmap: SN 100-102 ok) ─│
  │                                        │
  │  (删除 history[100..102])              │
```

HEARTBEAT 由 Writer **周期性**发送（默认几百 ms，可配），告知 Reader 当前缓存范围。Reader 据此生成 ACKNACK 确认/请求。

### 8.4.3 典型时序：丢失与重传

```
Writer                                   Reader
  │                                        │
  │── DATA (SN=100) ────────────────▶     │
  │── DATA (SN=101) ────X丢失              │
  │── DATA (SN=102) ────────────────▶     │
  │                                        │
  │── HEARTBEAT (first=100, last=102) ─▶  │
  │                                        │
  │   ◀─ ACKNACK (got 100, 102; missing 101) │
  │                                        │
  │── DATA (SN=101, 重传) ──────────▶     │
  │                                        │
  │   ◀─ ACKNACK (got 100-102) ───────────│
```

**关键**：Writer 在收到 ACKNACK 之前**必须在 history cache 保留消息**。
→ **Reliability 的代价**：更大的内存占用（history 保留）+ 更多网络往返（HB/ACK）。

### 8.4.4 history 满时的行为

Writer 的 `history.depth` 限制了能保留多少条：
- `KEEP_LAST(depth=10)`：仅保留最新 10 条；若 Reader 还没 ACK 旧的就被覆盖——**旧的丢了**（即使 RELIABLE）；
- `KEEP_ALL`：无上限，Writer 若写入速率 > Reader 消费速率 + 网络，**内存无限增长**直到 OOM。

这就是控制环用 `RELIABLE + KEEP_LAST(1)` 的原因：要最新的，不要旧的，不要 OOM。

---

## 8.5 HEARTBEAT 与 ACKNACK 周期

**默认 HEARTBEAT 周期**：
- FastDDS：100 ms（可通过 profile `<heartbeat_period>` 调）；
- CycloneDDS：100 ms（`CYCLONEDDS_URI` 里的 `<Heartbeat>`）。

**代价权衡**：
- 周期短 → 丢包检测快，重传及时，**带宽开销大**；
- 周期长 → 丢包被发现晚，用户感知到的延迟高，**带宽开销小**。

**RT 控制环的建议**：周期设为控制周期的 2–3 倍（如 500 Hz 用 4 ms HB）。

---

## 8.6 GAP 子消息

Writer 显式告诉 Reader："序号 X 到 Y 你别等了，我不发了"。常见场景：
- 消息被 history 淘汰（KEEP_LAST(10) 溢出）；
- Durability 过滤（VOLATILE 下新 Reader 不需要历史）；
- 消息被拒绝（权限 / content filter）。

**作用**：让 Reader 不停请求重传，加速 acknowledgedSN 前进。

---

## 8.7 分片（Fragmentation）

### 8.7.1 为什么需要

MTU 通常 1500 字节，去掉 IP/UDP/RTPS header 后**有效载荷 ~1400 字节**。一条 1 MB 消息要分 ~740 片。

### 8.7.2 DATA_FRAG

DATA 的分片版本，每片包含：
- `fragmentStartingNum`：本片在完整消息中的起始位置；
- `fragmentsInSubmessage`：本子消息含几个片；
- `fragmentSize`：每片字节数；
- `dataSize`：原始消息总字节数；
- `serializedPayload`：本片数据。

Reader 收集所有片后拼接。

### 8.7.3 HEARTBEAT_FRAG / NACK_FRAG

分片级心跳/确认。如果某条消息丢了一小片，不必重传整条——只重传那一片。

### 8.7.4 Fragment Size 调优

- 默认 1400 字节；
- 若走 SHM（MTU 无限），分片无意义——FastDDS 会自动检测并关闭分片；
- 跨千兆网 + Jumbo Frame（MTU 9000）→ 可调大 fragment size 到 ~8800，减少分片数，**大消息吞吐提升 30–50%**。

配置（FastDDS XML）：
```xml
<transport_descriptor>
  <sendBufferSize>4194304</sendBufferSize>
  <receiveBufferSize>4194304</receiveBufferSize>
  <maxMessageSize>65500</maxMessageSize>
</transport_descriptor>
```

---

## 8.8 Stateless vs Stateful Behavior

RTPS 定义两种 Writer/Reader 行为：

**Stateless Writer（Best-Effort）**：
- 不跟踪 Reader 状态；
- 发出 DATA 后即忘；
- 无 HEARTBEAT / ACKNACK；
- 开销最小，丢包不恢复。

**Stateful Writer（Reliable）**：
- 为每个匹配 Reader 维护 ReaderProxy；
- 跟踪 acknowledgedSN；
- 发 HEARTBEAT，处理 ACKNACK；
- 做重传。

QoS `reliability=BEST_EFFORT` 使用 Stateless 行为；`RELIABLE` 使用 Stateful。

**性能差异**：
- Stateless：发一条 DATA 即结束，单路径；
- Stateful：发 DATA + 周期性 HB + 可能的 repair DATA + GAP——**一条消息可能触发 3–5 个报文**。

---

## 8.9 发现协议（SPDP / SEDP）

发现本身也跑在 RTPS 之上：

### 8.9.1 SPDP（Simple Participant Discovery）

- 多播 DATA 子消息，payload 是 `ParticipantBuiltinTopicData`（描述本 Participant）；
- 默认每 3 秒一次；
- Participant 建立 "活着" 的视图。

### 8.9.2 SEDP（Simple Endpoint Discovery）

- SPDP 发现 Participant 后，通过内建的"发现 topic"（Reliable）交换 endpoint 元数据；
- `PublicationsWriter` / `SubscriptionsWriter` / `TopicsWriter` 等；
- 交换完成后，应用层 Writer/Reader 才能匹配。

**所以发现期本身就是 RTPS 流量**——RTPS 的 overhead 包含**应用数据 + 发现数据**。

---

## 8.10 安全（DDS Security）

启用 security 时：
- DATA payload 加密（AES-GCM）；
- 每个 submessage 附加 MAC；
- 密钥通过"参与者认证握手"建立；
- RTPS Header 不加密（需要明文路由信息）。

**性能代价**：RELIABLE + Security 下吞吐降 20–40%，延迟增 30 μs+。

---

## 8.11 实战：用 Wireshark 看 RTPS

Wireshark 原生支持 RTPS 解析：

```bash
sudo tcpdump -i lo -w trace.pcap udp port 7400-7500
# 让节点跑一会
wireshark trace.pcap
```

Display filter：
- `rtps` —— 所有 RTPS；
- `rtps.sm.wrEntityId == 0xc2` —— Publications writer（SEDP）；
- `rtps.writer_seq_number > 1000` —— 特定范围；

能看到：
- SPDP 广播（每 3 s）；
- SEDP 交换（发现期密集）；
- DATA + HEARTBEAT + ACKNACK 循环；
- GAP / DATA_FRAG（大消息时）。

**排查丢包** / **延迟定位** 必备技能。

---

## 8.12 FastDDS / CycloneDDS 行为差异

| 维度 | FastDDS | CycloneDDS |
|------|---------|-----------|
| HB 调度 | 基于定时器 | 基于 DATA 间隔 |
| ACKNACK 合并 | 有（延迟发送，合并多个） | 有 |
| 传输并行 | 多线程（per transport） | 单 reactor 线程 |
| 分片并行 | 多线程 | 单线程 |
| Discovery 可选 | SDP 或 Discovery Server | SDP 或 static peers |

**对延迟敏感场景的建议**：CycloneDDS 的单 reactor 线程模型抖动更低；FastDDS 的多线程在吞吐场景更强。

---

## 8.13 小结

- RTPS 是 DDS 的 wire protocol，由 Header + N × Submessage 组成；
- 可靠性通过 HEARTBEAT/ACKNACK 循环实现——代价是内存 + 带宽 + 延迟；
- 大消息走 DATA_FRAG，MTU 决定分片效率；
- 发现（SPDP/SEDP）也跑在 RTPS 之上，一起消耗资源；
- Wireshark + `rtps` 过滤器是最强的调试工具；
- QoS 的"神秘"行为多源于 RTPS 机制——知道了机制，就能做合理权衡。

下一章：[09 SHM 传输](./09_shm_transport.md)。
