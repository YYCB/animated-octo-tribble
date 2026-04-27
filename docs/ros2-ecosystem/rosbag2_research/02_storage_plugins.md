# 存储插件深度对比

> **参考来源**：`ros2/rosbag2@humble`：`rosbag2_storage_mcap/`、`rosbag2_storage_sqlite3/`、`rosbag2_compression/`

---

## 一、存储插件架构

rosbag2 通过 `pluginlib` 实现存储后端的热插拔。当前主要支持两种存储格式：

| 特性 | MCAP | SQLite3 |
|------|------|---------|
| 文件格式 | 自描述二进制容器 | 关系型数据库 |
| 默认版本（Humble） | ✅ 默认（humble 起） | 遗留（Foxy/Galactic 默认） |
| seek 性能 | O(1)（Summary Section 索引）| O(n)（全表扫描或 timestamp 索引） |
| 多 topic 查询 | 单文件多通道 | 单表多行 |
| 工具生态 | Foxglove、mcap CLI、RViz | sqlite3 CLI |
| 大文件支持 | 优秀（chunk 分段）| 受 SQLite WAL 限制 |
| 流式写入 | ✅ chunk 级 flush | ✅ WAL 模式 |
| 消息定义存储 | ✅ Schema section | ✅ topics 表 |
| 跨语言支持 | C/C++/Python/JS/Go SDK | SQLite 驱动 |

---

## 二、MCAP 格式深度解析

### 2.1 文件结构概览

MCAP（Message Capture）是 Foxglove 主导的开放标准格式，专为机器人日志设计。

```
MCAP 文件布局（顺序结构）：
┌──────────────────────────────────────────────────────────────┐
│  Magic: 0x89 MCAP 0x0D 0x0A 0x1A 0x0A  (8 bytes)            │
├──────────────────────────────────────────────────────────────┤
│  Header Record                                               │
│  ├─ profile: "ros2"                                          │
│  └─ library: "rosbag2_storage_mcap/..."                      │
├──────────────────────────────────────────────────────────────┤
│  Data Section                                                │
│  ├─ Schema Record(s)        ← 消息类型定义                    │
│  │    schemaId, name, encoding, data                         │
│  ├─ Channel Record(s)       ← topic 通道定义                  │
│  │    channelId, schemaId, topic, messageEncoding            │
│  ├─ Chunk Record(s)         ← 数据块（包含多条消息）            │
│  │  ┌─────────────────────────────────────────────────┐     │
│  │  │  Chunk Header                                    │     │
│  │  │  ├─ messageStartTime / messageEndTime           │     │
│  │  │  ├─ uncompressedSize / uncompressedCrc           │     │
│  │  │  └─ compression: "" / "zstd" / "lz4"            │     │
│  │  │  Message Record(s)                               │     │
│  │  │  ├─ channelId, sequence, logTime, publishTime   │     │
│  │  │  └─ data (CDR bytes)                             │     │
│  │  └─────────────────────────────────────────────────┘     │
│  └─ Message Index Record(s) ← per-chunk per-channel 索引    │
│       channelId → [(timestamp, offset), ...]                 │
├──────────────────────────────────────────────────────────────┤
│  Data End Record                                             │
├──────────────────────────────────────────────────────────────┤
│  Summary Section                                             │
│  ├─ Statistics Record       ← 全局统计（messageCount 等）     │
│  ├─ Schema Record(s)        ← 重复（汇总用）                  │
│  ├─ Channel Record(s)       ← 重复（汇总用）                  │
│  └─ Chunk Index Record(s)   ← 每个 Chunk 的时间范围 + 偏移量  │
│       chunkStartOffset, chunkLength,                         │
│       messageStartTime, messageEndTime,                      │
│       messageIndexOffset(s)                                  │
├──────────────────────────────────────────────────────────────┤
│  Summary Offset Record                                        │
│  Footer Record                                               │
│  ├─ summaryStart (file offset)                               │
│  └─ summaryOffsetStart                                       │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 时间范围查询原理（O(1) seek）

MCAP 的 Chunk Index 存储在文件末尾的 Summary Section，**不需要扫描整个文件**即可定位时间段：

```
查询流程（seek to timestamp T）：

1. 读取文件末尾 Footer → 获取 summaryStart 偏移量
2. 跳转到 summaryStart → 读取 Chunk Index Records
   每条 ChunkIndex = {
     messageStartTime: T_start,
     messageEndTime:   T_end,
     chunkStartOffset: file_offset,
     chunkLength:      chunk_byte_size,
     messageIndexOffsets: {channelId → index_offset}
   }

3. 二分查找 ChunkIndex（按 messageStartTime 排序）：
   找到第一个 T_end >= T 的 Chunk

4. 直接跳转到 chunkStartOffset 读取 Chunk
5. 在 Chunk 内部的 Message Index 中二分查找具体消息偏移量

时间复杂度：O(log C)，C = Chunk 数量
空间开销：Summary Section ≈ 0.01% 文件大小
```

### 2.3 Chunk 大小与写入策略

```yaml
# rosbag2_storage_mcap 的默认配置
storage:
  storage_id: mcap
  storage_config_uri: ""  # 可指向自定义配置文件

# 通过 custom_data 传递给 mcap 插件的参数：
custom_data:
  compression: "zstd"          # chunk 级压缩
  chunk_size: "786432"         # 768KB per chunk（默认值）
  # 注意：chunk_size 过小 → 大量 chunk index 开销
  #        chunk_size 过大 → seek 粒度粗糙
```

**具身智能场景推荐**：
- 高频关节（1kHz）+ 图像（30fps）混合录制：`chunk_size = 4MB`
- 单纯图像录制（30fps H=720p）：`chunk_size = 8-16MB`（一帧一 chunk 效率低）

---

## 三、SQLite3 格式深度解析

### 3.1 数据库 Schema

```sql
-- rosbag2_storage_sqlite3 使用的 schema（版本 3）
CREATE TABLE IF NOT EXISTS schema (
  schema_version INTEGER PRIMARY KEY,
  ros_distro TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS metadata (
  id INTEGER PRIMARY KEY,
  metadata_version INTEGER NOT NULL,
  metadata TEXT NOT NULL  -- JSON 序列化的 BagMetadata
);

CREATE TABLE IF NOT EXISTS topics (
  id INTEGER PRIMARY KEY,
  name TEXT NOT NULL,
  type TEXT NOT NULL,
  serialization_format TEXT NOT NULL,
  offered_qos_profiles TEXT NOT NULL,
  type_description_hash TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS messages (
  id INTEGER PRIMARY KEY,
  topic_id INTEGER NOT NULL,
  timestamp INTEGER NOT NULL,  -- nanoseconds since epoch
  data BLOB NOT NULL,          -- CDR 序列化字节流
  FOREIGN KEY(topic_id) REFERENCES topics(id)
);

-- 时间戳索引（加速时间范围查询）
CREATE INDEX IF NOT EXISTS timestamp_idx ON messages (timestamp ASC);
```

### 3.2 WAL 模式与写入性能

SQLite3 默认使用 WAL（Write-Ahead Logging）模式，写入流程：

```
写入操作：
  1. 消息追加到 WAL 文件（wal_file.db3-wal）→ 顺序写，极快
  2. 读操作同时查询 main db 和 WAL → 保证一致性
  3. WAL checkpoint（触发条件：WAL 文件 > 1000 pages 或 PRAGMA wal_checkpoint）
     → 将 WAL 内容合并回主数据库文件

rosbag2_storage_sqlite3 的关键 PRAGMA 设置：
  PRAGMA journal_mode=WAL
  PRAGMA synchronous=NORMAL    -- 非 FULL，减少 fsync 开销
  PRAGMA cache_size=-8192      -- 8MB 内存缓存
  PRAGMA page_size=4096        -- 4KB 页大小
```

**SQLite3 性能上限**（Humble，典型环境）：

| 场景 | 写入速率上限 | 限制因素 |
|------|------------|---------|
| 纯关节状态（1kHz，~200B/msg）| ~5,000 msg/s | INSERT 行锁 |
| 混合 topic（关节+图像）| ~300 MB/s（SSD）| WAL checkpoint 暂停 |
| 关键瓶颈 | - | 每条消息一次 INSERT，事务提交频率 |

**改善 SQLite3 写入性能**：
```yaml
# 通过 storage_config_uri 指向 yaml 文件
pragma:
  journal_mode: wal
  synchronous: "0"        # OFF（危险，断电可能丢数据）
  cache_size: -65536       # 64MB 缓存
  write_batch_size: 100    # 批量写入（rosbag2 1.x 新增）
```

### 3.3 SQLite3 的适用场景

**推荐使用 SQLite3 的场景**：
- 低频 topic 录制（< 100 msg/s）
- 需要 SQL 查询能力的离线分析
- 旧版 ROS 2（Foxy/Galactic）兼容性需求

**不推荐使用 SQLite3 的场景**：
- 高频图像录制（30fps × 2MB/帧 = 60 MB/s）
- 需要快速随机访问（seek 依赖 timestamp 索引查询，比 mcap 慢 5-10x）
- 超大 bag 文件（> 10GB，SQLite 文件大小性能下降明显）

---

## 四、压缩插件详解

### 4.1 压缩模式对比

| 模式 | 参数值 | 工作方式 | 适用场景 |
|------|--------|---------|---------|
| per-message | `compression_mode: "message"` | 每条消息独立压缩/解压 | 需要随机访问单条消息 |
| per-file | `compression_mode: "file"` | 关闭 bag 后整体压缩 | 最高压缩比，不支持边录边压 |
| per-chunk（mcap 原生）| 通过 mcap plugin custom_data | 每个 Chunk 独立压缩 | **推荐**：平衡压缩比与 seek 性能 |
| 无压缩 | `compression_mode: ""` | 原始 CDR 字节 | 最高写入吞吐 |

### 4.2 zstd vs lz4 性能对比

以下数据基于典型具身智能录制场景（混合关节+RGB-D图像数据）：

| 指标 | zstd (level 1) | zstd (level 3) | lz4 |
|------|---------------|----------------|-----|
| 压缩比（关节状态）| 3.2x | 4.1x | 2.1x |
| 压缩比（RGB图像）| 1.4x | 1.6x | 1.2x |
| 压缩比（深度图像）| 2.8x | 3.5x | 2.0x |
| 压缩 CPU（单核）| ~150 MB/s | ~60 MB/s | ~400 MB/s |
| 解压 CPU（单核）| ~600 MB/s | ~600 MB/s | ~1000 MB/s |
| 适用场景 | 长期归档存储 | 最高压缩比 | 实时录制（低延迟）|

**推荐配置**：
```yaml
# 实时录制（优先写入吞吐）
storage:
  compression_format: "lz4"
  compression_mode: "message"  # mcap chunk 级压缩（通过 custom_data）

# 离线归档（优先存储空间）
storage:
  compression_format: "zstd"
  compression_mode: "file"     # 录制完成后压缩整个文件
  compression_threads: 4       # 多线程压缩
```

### 4.3 压缩对图像数据的效果

图像（sensor_msgs/Image）已经是**原始像素数据**（未压缩 RGB/BGR/深度），通用压缩效果有限：

- **RGB 图像**（640×480×3 = 921,600 bytes/帧）：lz4 压缩率约 10-20%
- **深度图像**（640×480×2 = 614,400 bytes/帧，uint16）：lz4 压缩率约 30-40%
- **关节状态**（~500 bytes/帧）：lz4 压缩率约 50-60%

**替代方案**：使用 `image_transport` 的 compressed 或 theora 插件，在发布端就进行图像压缩，记录压缩后的 topic（`/camera/color/image_raw/compressed`）：

```yaml
# 录制压缩图像 topic（节省约 80% 存储空间）
record:
  topics:
    - /camera/color/image_raw/compressed    # JPEG/PNG 压缩
    - /camera/depth/image_raw/compressedDepth  # 深度专用压缩
```

---

## 五、分片（Bag Splitting）机制

### 5.1 分片配置

```yaml
# 按大小分片
storage:
  max_bagfile_size: 1073741824   # 1GB（字节）

# 按时长分片
storage:
  max_bagfile_duration: 300000000000  # 300秒（纳秒）

# 两者同时设置：任一条件触发即分片
```

### 5.2 分片命名规则

```
base_dir/
├── my_bag_0.mcap        ← 第一个分片
├── my_bag_1.mcap        ← 第二个分片
├── my_bag_2.mcap        ← 第三个分片
└── metadata.yaml        ← 统一索引（列出所有分片）
```

分片的 `metadata.yaml` 示例：
```yaml
rosbag2_bagfile_information:
  relative_file_paths:
    - my_bag_0.mcap
    - my_bag_1.mcap
    - my_bag_2.mcap
  # 各分片共享同一个 metadata，读取时自动跨分片连续读取
```

### 5.3 分片的 Episode 对齐

具身智能场景中，**分片与 episode 边界对齐**非常重要：

```python
# 利用 BagSplitCallback 在 episode 开始时触发分片
from rosbag2_cpp import Writer

class EpisodeAlignedWriter:
    def __init__(self):
        self.writer = Writer()
        self.writer.add_event_callbacks(
            bag_split_callback=self._on_bag_split
        )

    def start_new_episode(self):
        """在新 episode 开始时手动分片"""
        # 方法1：通过 Service 调用（CLI）
        # ros2 service call /rosbag2_recorder/split_bagfile std_srvs/srv/Trigger

        # 方法2：监听 /episode_marker topic 并触发分片
        pass

    def _on_bag_split(self, bag_split_info):
        print(f"New bag file: {bag_split_info.closed_file}")
```

---

## 六、索引与随机访问对比

### 6.1 MCAP O(1) seek 实现

```
MCAP seek(T) 流程：

Step 1: read Footer（8 bytes from end）
         → summaryStart = 文件末尾偏移量

Step 2: seek(summaryStart)，读取所有 ChunkIndex records
         内存中排好序：[(T_start_0, T_end_0, offset_0), ...]
         时间复杂度：O(C)，C = Chunk 数量（通常 < 1000）

Step 3: binary_search for first ChunkIndex where T_end >= T
         时间复杂度：O(log C)

Step 4: seek(chunk_offset)，读取 Chunk Header
         解压（如有）→ 二分查找 Message Index
         时间复杂度：O(log M_per_chunk)

总计：O(log C + log M)，实际 < 1ms（对 30GB 文件也适用）
```

### 6.2 SQLite3 seek 实现

```sql
-- SequentialReader::seek(T) 的底层 SQL：
SELECT id, topic_id, timestamp, data
FROM messages
WHERE timestamp >= ?
ORDER BY timestamp ASC
LIMIT ?

-- 有 timestamp_idx 索引时：B-tree 查找，O(log N)
-- 但每次 read_next() 依然需要 one SQL round-trip
-- 大文件（> 1M 条消息）时索引查找本身也慢
```

**seek 性能实测对比**（100万条消息，10GB bag）：

| 操作 | MCAP | SQLite3（有索引）|
|------|------|----------------|
| 首次 seek（冷缓存）| 2ms | 150ms |
| 连续 seek（不同时间点）| 1ms | 80ms |
| read_next()（顺序）| 0.01ms | 0.05ms |
| get_metadata()（bag 信息）| 0.5ms | 50ms |

---

## 七、具身智能采集建议

### 7.1 标准采集场景带宽估算

典型具身智能机械臂采集（6 DoF + RGB-D + IMU）：

| 数据源 | 频率 | 单帧大小 | 带宽 |
|--------|------|---------|------|
| `/joint_states`（6 DoF）| 1kHz | ~200 B | 0.2 MB/s |
| `/joint_commands`（6 DoF）| 1kHz | ~200 B | 0.2 MB/s |
| `/camera/color/image_raw`（720p RGB）| 30fps | 1.38 MB | 41.5 MB/s |
| `/camera/depth/image_raw`（720p uint16）| 30fps | 0.92 MB | 27.6 MB/s |
| `/imu/data` | 200Hz | ~100 B | 0.02 MB/s |
| `/wrench`（6轴力矩）| 500Hz | ~100 B | 0.05 MB/s |
| `/tf`（TF 树快照）| 50Hz | ~500 B | 0.025 MB/s |
| **合计**（未压缩）| - | - | **~69.6 MB/s** |
| **合计**（lz4 压缩后估算）| - | - | **~40-55 MB/s** |

### 7.2 存储选型决策

```
根据采集速率选择存储格式：

写入速率 < 10 MB/s（纯传感器，无图像）
  → sqlite3（方便 SQL 分析）或 mcap（统一格式）

写入速率 10-100 MB/s（含单个相机）
  → mcap + lz4（chunk 级压缩）+ NVMe SSD

写入速率 > 100 MB/s（多相机 + 高频关节）
  → mcap（无压缩）+ NVMe RAID 或 RAM buffer → 异步落盘
  → 考虑在录制后离线压缩（zstd level 3）

存储空间有限（<100GB/天预算）
  → 录制压缩图像 topic（image_transport compressed）
  → 原始关节/力矩数据不压缩（本身已很小）
```

### 7.3 推荐配置模板

```yaml
# 具身智能高可靠录制配置（推荐用于真机采集）
storage:
  storage_id: mcap
  max_bagfile_size: 2147483648   # 2GB 分片（平衡管理性与连续性）
  max_cache_size: 524288000      # 500MB 写缓存（减少 I/O 频率）
  compression_format: ""         # 写入时不压缩（保证吞吐）
  storage_preset_profile: "resilient"  # 开启数据完整性校验

record:
  topics:
    - /joint_states
    - /camera/color/image_raw/compressed   # 使用 compressed 节省空间
    - /camera/depth/image_raw/compressedDepth
    - /imu/data
    - /wrench
    - /tf
    - /tf_static
    - /episode_marker
  rmw_serialization_format: cdr
  topic_polling_interval: 100    # ms，topic 发现轮询间隔
  qos_profile_overrides_path: /config/qos_overrides.yaml

# 录制完成后离线归档（可选）：
# ros2 bag convert -i my_bag -o my_bag_compressed \
#   --storage-config-uri compress_config.yaml
```
