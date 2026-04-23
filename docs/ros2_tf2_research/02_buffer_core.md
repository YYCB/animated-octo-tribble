# tf2（二）：BufferCore 源码深度解析

## 一、setTransform() — 写入变换

```cpp
// tf2/src/buffer_core.cpp

bool BufferCore::setTransform(
  const geometry_msgs::msg::TransformStamped & transform_in,
  const std::string & authority,
  bool is_static)
{
  // ★ Step 1: 输入验证
  if (!validateQuaternion(transform_in.transform.rotation)) {
    throw tf2::InvalidArgumentException(
      "Quaternion is not normalized! ...");
  }
  
  if (transform_in.child_frame_id == transform_in.header.frame_id) {
    throw tf2::InvalidArgumentException(
      "Frame cannot have itself as its parent!");
  }
  
  // ★ Step 2: 字符串帧名 → CompactFrameID（整数）
  CompactFrameID frame_id = lookupOrInsertFrameNumber(
    transform_in.child_frame_id);
  CompactFrameID parent_id = lookupOrInsertFrameNumber(
    transform_in.header.frame_id);
  
  // ★ Step 3: 转换为内部 TransformStorage
  TransformStorage storage;
  storage.stamp_ = tf2_ros::fromMsg(transform_in.header.stamp);
  storage.rotation_ = tf2::impl::fromMsg(transform_in.transform.rotation);
  storage.translation_ = tf2::impl::fromMsg(transform_in.transform.translation);
  storage.frame_id_ = parent_id;
  storage.child_frame_id_ = frame_id;
  
  // ★ Step 4: 写入时间缓存（加锁）
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    
    // 确保 frames_ 向量足够大
    if (frame_id >= frames_.size()) {
      frames_.resize(frame_id + 1);
    }
    if (!frames_[frame_id]) {
      // 首次创建：为此帧分配 TimeCache
      frames_[frame_id] = std::make_shared<TimeCache>(cache_time_);
    }
    
    // ★ 插入变换到时间缓存
    bool ok = frames_[frame_id]->insertData(storage);
    
    if (!ok) {
      // 时间戳比缓存最旧数据还旧，丢弃
      return false;
    }
  }
  
  // ★ Step 5: 通知等待中的 waitForTransform 调用
  {
    std::unique_lock<std::mutex> lock(transforms_available_mutex_);
    transforms_available_cond_.notify_all();
  }
  
  return true;
}
```

---

## 二、TimeCache::insertData() — 有序插入

```cpp
// tf2/src/time_cache.cpp

bool TimeCache::insertData(const TransformStorage & new_data)
{
  // storage_ 是时间有序的双端队列（最新在前）
  
  // 如果新数据比最旧数据还旧（超出缓存窗口），丢弃
  if (!storage_.empty()) {
    auto oldest_time = storage_.back().stamp_;
    if (new_data.stamp_ + cache_time_ < oldest_time) {
      return false;
    }
  }
  
  // ★ 找到正确的插入位置（保持时间有序）
  auto it = storage_.begin();
  while (it != storage_.end()) {
    if (it->stamp_ <= new_data.stamp_) break;
    ++it;
  }
  storage_.insert(it, new_data);
  
  // ★ 清除过期数据（超过 cache_time 的老数据）
  pruneList();
  
  return true;
}

void TimeCache::pruneList()
{
  auto latest_time = storage_.front().stamp_;
  
  // 从尾部移除超过 cache_time 的数据
  while (!storage_.empty()) {
    if (latest_time - storage_.back().stamp_ > cache_time_) {
      storage_.pop_back();
    } else {
      break;
    }
  }
}
```

---

## 三、lookupTransform() 完整调用链

```cpp
// tf2/src/buffer_core.cpp

geometry_msgs::msg::TransformStamped
BufferCore::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time) const
{
  // ★ Step 1: 帧名 → ID
  CompactFrameID target_id = lookupFrameNumber(target_frame);
  CompactFrameID source_id = lookupFrameNumber(source_frame);
  
  if (target_id == 0 || source_id == 0) {
    throw LookupException("Frame does not exist: ...");
  }
  
  // ★ Step 2: 核心查找
  tf2::Transform output;
  tf2::TimePoint time_out;
  
  std::string error_string;
  TF2Error retval = lookupTransformImpl(
    target_id, source_id, time, output, time_out, error_string);
  
  if (retval != TF2Error::NO_ERROR) {
    throw makeException(error_string, retval);
  }
  
  // ★ Step 3: 转换为 geometry_msgs
  return tf2::toMsg(tf2::Stamped<tf2::Transform>(output, time_out, target_frame));
}
```

---

## 四、lookupTransformImpl() — LCA 算法核心

```cpp
// tf2/src/buffer_core.cpp

TF2Error
BufferCore::lookupTransformImpl(
  CompactFrameID target_id,
  CompactFrameID source_id,
  const tf2::TimePoint & time,
  tf2::Transform & output,
  tf2::TimePoint & time_out,
  std::string & error_string) const
{
  // ★ Step 1: 分别从 source 和 target 向根节点走，找到 LCA
  
  // 从 source 向上走，收集路径
  TransformAccum source_accum;
  TF2Error rv = walkToTopParent(source_accum, time, source_id,
                                 target_id, &error_string);
  
  // walkToTopParent 同时从两端走，找 LCA：
  // 1. 用两个指针分别在 source_path 和 target_path 中前进
  // 2. 记录访问过的帧（用 frame_set）
  // 3. 发现某帧已在另一路径的 frame_set 中 → 此帧为 LCA
  
  // ★ Step 2: 合并变换
  // source_accum 内已收集从 source 到 LCA 的逆变换链
  // 再与从 LCA 到 target 的变换链相乘
  
  output = source_accum.result;  // 最终变换矩阵
  time_out = source_accum.time;
  
  return TF2Error::NO_ERROR;
}
```

---

## 五、walkToTopParent() — 爬树核心

```cpp
template<typename F>
TF2Error
BufferCore::walkToTopParent(
  F & f,
  tf2::TimePoint time,
  CompactFrameID target_id,
  CompactFrameID source_id,
  std::string * error_string) const
{
  // ★ 双指针同步向上走
  CompactFrameID frame = source_id;
  CompactFrameID top_node = source_id;
  
  // 记录 target 侧已访问的帧（O(depth) 空间）
  CompactFrameID target_frame = target_id;
  
  // 构建 target → root 的帧 ID 集合
  std::vector<CompactFrameID> target_path;
  while (target_frame != 0) {
    target_path.push_back(target_frame);
    target_frame = getParentFrame(target_frame);
  }
  std::set<CompactFrameID> target_set(target_path.begin(), target_path.end());
  
  // 从 source 向上走，直到遇到 target 路径上的帧（LCA）
  while (frame != 0) {
    if (target_set.count(frame)) {
      // ★ 找到 LCA！
      top_node = frame;
      break;
    }
    
    // ★ 从当前帧的 TimeCache 中取时间插值后的变换
    tf2::Transform transform;
    TF2Error retval = getLatestCommonTime(frame, transform, time, error_string);
    
    if (retval != TF2Error::NO_ERROR) return retval;
    
    // ★ 调用仿函数累积变换（TransformAccum）
    f.accum(transform);
    
    // 向上走一步
    frame = getParentFrame(frame);
  }
  
  // 从 LCA 走向 target（需要取逆变换）
  frame = target_id;
  while (frame != top_node) {
    tf2::Transform transform;
    getLatestCommonTime(frame, transform, time, error_string);
    
    // ★ 取逆（child→parent 变成 parent→child）
    f.accum(transform.inverse());
    
    frame = getParentFrame(frame);
  }
  
  f.finalize();
  return TF2Error::NO_ERROR;
}
```

---

## 六、TimeCache::getData() — 时间插值

```cpp
// tf2/src/time_cache.cpp

TF2Error
TimeCache::getData(
  tf2::TimePoint time,
  TransformStorage & data_out,
  std::string * error_str) const
{
  // ★ 特殊情况：时间为 0 → 取最新数据
  if (time == tf2::TimePointZero) {
    data_out = storage_.front();
    return TF2Error::NO_ERROR;
  }
  
  // ★ 找到时间区间 [t0, t1]，满足 t0 >= time >= t1
  auto one = storage_.end();
  auto two = storage_.end();
  
  for (auto it = storage_.begin(); it != storage_.end(); ++it) {
    if (it->stamp_ <= time) {
      two = it;
      if (it != storage_.begin()) {
        one = std::prev(it);
      }
      break;
    }
  }
  
  if (one == storage_.end() || two == storage_.end()) {
    // 超出范围：ExtrapolationException
    return TF2Error::EXTRAPOLATION_ERROR;
  }
  
  // ★ SLERP 插值（如果时间精确匹配则直接返回）
  if (one->stamp_ == time) {
    data_out = *one;
    return TF2Error::NO_ERROR;
  }
  
  tf2Scalar ratio = tf2Scalar(
    tf2::durationToSec(time - two->stamp_) /
    tf2::durationToSec(one->stamp_ - two->stamp_)
  );
  
  // 平移：线性插值
  data_out.translation_ = lerp(two->translation_, one->translation_, ratio);
  
  // 旋转：SLERP（球面线性插值）
  data_out.rotation_ = slerp(two->rotation_, one->rotation_, ratio);
  
  data_out.stamp_ = time;
  data_out.frame_id_ = one->frame_id_;
  data_out.child_frame_id_ = one->child_frame_id_;
  
  return TF2Error::NO_ERROR;
}
```

---

## 七、canTransform() — 非阻塞检查

```cpp
// tf2/src/buffer_core.cpp

bool BufferCore::canTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  std::string * error_msg) const
{
  CompactFrameID target_id = lookupFrameNumber(target_frame);
  CompactFrameID source_id = lookupFrameNumber(source_frame);
  
  if (target_id == 0 || source_id == 0) {
    if (error_msg) *error_msg = "Frame does not exist";
    return false;
  }
  
  // ★ 用 CanTransformAccum（只检查，不计算变换）
  CanTransformAccum accum;
  TF2Error retval = walkToTopParent(accum, time, target_id, source_id, error_msg);
  
  return retval == TF2Error::NO_ERROR;
}
```

---

## 八、高级查询：lookupTransform with timeout

```cpp
// tf2_ros/src/buffer.cpp
// Buffer 继承 BufferCore，增加异步等待功能

geometry_msgs::msg::TransformStamped
Buffer::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout) const
{
  // ★ 等待变换可用（使用 condition_variable）
  if (!canTransform(target_frame, source_frame, time)) {
    if (!waitForTransform(target_frame, source_frame, time, timeout)) {
      throw LookupException("Transform not available within timeout");
    }
  }
  
  // 变换已可用，直接查询
  return BufferCore::lookupTransform(target_frame, source_frame, time);
}

// waitForTransform 内部：
bool Buffer::waitForTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout,
  std::string * errstr) const
{
  auto start = tf2::get_now();
  
  // 每 10ms 轮询一次
  while (tf2::get_now() - start < timeout) {
    if (canTransform(target_frame, source_frame, time, errstr)) {
      return true;
    }
    // ★ 等待 transforms_available_cond_ 通知（setTransform 会 notify）
    std::unique_lock<std::mutex> lock(transforms_available_mutex_);
    transforms_available_cond_.wait_for(lock, std::chrono::milliseconds(10));
  }
  return false;
}
```
