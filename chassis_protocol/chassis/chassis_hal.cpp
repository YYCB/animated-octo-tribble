#include "chassis_hal.hpp"

#include <chrono>
#include <cstring>
#include <thread>

namespace chassis {

// ─── Little-endian helpers ────────────────────────────────────────────────────

static void appendU8(std::vector<uint8_t>& buf, uint8_t v) {
    buf.push_back(v);
}

static void appendU16LE(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

static void appendU32LE(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

static void appendU64LE(std::vector<uint8_t>& buf, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        buf.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
    }
}

static void appendDouble(std::vector<uint8_t>& buf, double v) {
    uint64_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    appendU64LE(buf, bits);
}

static uint64_t nowUs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

// ─── Read helpers for uplink payloads ────────────────────────────────────────

static uint32_t readU32LE(const std::vector<uint8_t>& p, std::size_t& off) {
    if (off + 4 > p.size()) { off = p.size(); return 0; }
    uint32_t v = static_cast<uint32_t>(p[off])
               | (static_cast<uint32_t>(p[off+1]) << 8)
               | (static_cast<uint32_t>(p[off+2]) << 16)
               | (static_cast<uint32_t>(p[off+3]) << 24);
    off += 4;
    return v;
}

static uint64_t readU64LE(const std::vector<uint8_t>& p, std::size_t& off) {
    if (off + 8 > p.size()) { off = p.size(); return 0; }
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        v |= (static_cast<uint64_t>(p[off + i]) << (8 * i));
    }
    off += 8;
    return v;
}

static uint16_t readU16LE(const std::vector<uint8_t>& p, std::size_t& off) {
    if (off + 2 > p.size()) { off = p.size(); return 0; }
    uint16_t v = static_cast<uint16_t>(p[off]) | (static_cast<uint16_t>(p[off+1]) << 8);
    off += 2;
    return v;
}

static uint8_t readU8(const std::vector<uint8_t>& p, std::size_t& off) {
    if (off >= p.size()) return 0;
    return p[off++];
}

static double readDouble(const std::vector<uint8_t>& p, std::size_t& off) {
    uint64_t bits = readU64LE(p, off);
    double v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

// ─── ChassisHal ──────────────────────────────────────────────────────────────

ChassisHal::ChassisHal(std::unique_ptr<ITransport> transport, uint32_t protocol_version)
    : transport_(std::move(transport))
    , protocol_version_(protocol_version)
    , negotiator_(*transport_, protocol_version)
{
    transport_->setReceiveCallback(
        [this](const uint8_t* data, std::size_t len) {
            onDataReceived(data, len);
        });
}

ChassisHal::~ChassisHal() {
    disconnect();
}

bool ChassisHal::connect() {
    if (connected_.load()) {
        return true;
    }

    if (!transport_->connect()) {
        return false;
    }

    negotiator_.reset();
    if (!negotiator_.sendHandshake()) {
        transport_->disconnect();
        return false;
    }

    // Wait up to 6 seconds for negotiation
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(6);
    while (std::chrono::steady_clock::now() < deadline) {
        if (negotiator_.isNegotiated()) {
            break;
        }
        if (negotiator_.checkTimeout()) {
            // Exceeded max retries
            transport_->disconnect();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!negotiator_.isNegotiated()) {
        transport_->disconnect();
        return false;
    }

    connected_.store(true);
    running_.store(true);
    reconnect_thread_ = std::thread(&ChassisHal::reconnectLoop, this);

    // Populate info from negotiator
    {
        std::lock_guard<std::mutex> lk(status_mutex_);
        info_.protocol_version = negotiator_.getNegotiatedVersion();
        info_.model            = negotiator_.getChassisType();
    }

    return true;
}

void ChassisHal::disconnect() {
    running_.store(false);
    connected_.store(false);

    if (transport_) {
        transport_->disconnect();
    }

    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
}

bool ChassisHal::isConnected() const {
    return connected_.load();
}

uint32_t ChassisHal::nextSeq() {
    return seq_++;
}

// ─── Send helpers ─────────────────────────────────────────────────────────────

std::vector<uint8_t> ChassisHal::buildVelocityPayload(double vx, double vy, double omega) {
    // seq(4) + vx(8) + vy(8) + omega(8) + timestamp_us(8) = 36 bytes
    std::vector<uint8_t> buf;
    buf.reserve(36);
    appendU32LE(buf, nextSeq());
    appendDouble(buf, vx);
    appendDouble(buf, vy);
    appendDouble(buf, omega);
    appendU64LE(buf, nowUs());
    return buf;
}

std::vector<uint8_t> ChassisHal::buildLightPayload(LightMode mode, uint8_t r, uint8_t g,
                                                    uint8_t b, double hz) {
    // mode(1) + r(1) + g(1) + b(1) + blink_hz(8) = 12 bytes
    std::vector<uint8_t> buf;
    buf.reserve(12);
    appendU8(buf, static_cast<uint8_t>(mode));
    appendU8(buf, r);
    appendU8(buf, g);
    appendU8(buf, b);
    appendDouble(buf, hz);
    return buf;
}

std::vector<uint8_t> ChassisHal::buildStopPayload(const std::string& reason) {
    // reason_len(2) + reason(N)
    std::vector<uint8_t> buf;
    const uint16_t len = static_cast<uint16_t>(reason.size());
    buf.reserve(2 + len);
    appendU16LE(buf, len);
    for (char c : reason) {
        buf.push_back(static_cast<uint8_t>(c));
    }
    return buf;
}

// ─── IChassisController implementation ───────────────────────────────────────

bool ChassisHal::sendVelocity(double vx, double vy, double omega) {
    if (!connected_.load()) return false;
    auto payload = buildVelocityPayload(vx, vy, omega);
    auto frame   = FrameCodec::encode(CmdId::VELOCITY_CMD, payload);
    return transport_->send(frame.data(), frame.size());
}

bool ChassisHal::sendLightCmd(LightMode mode, uint8_t r, uint8_t g, uint8_t b, double blink_hz) {
    if (!connected_.load()) return false;
    auto payload = buildLightPayload(mode, r, g, b, blink_hz);
    auto frame   = FrameCodec::encode(CmdId::LIGHT_CMD, payload);
    return transport_->send(frame.data(), frame.size());
}

bool ChassisHal::emergencyStop(const std::string& reason) {
    if (!connected_.load()) return false;
    auto payload = buildStopPayload(reason);
    auto frame   = FrameCodec::encode(CmdId::EMERGENCY_STOP, payload);
    return transport_->send(frame.data(), frame.size());
}

ChassisStatus ChassisHal::getStatus() const {
    std::lock_guard<std::mutex> lk(status_mutex_);
    return current_status_;
}

ChassisInfo ChassisHal::getInfo() const {
    std::lock_guard<std::mutex> lk(status_mutex_);
    return info_;
}

OdometryData ChassisHal::getOdometry() const {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    return current_odom_;
}

BatteryData ChassisHal::getBatteryInfo() const {
    std::lock_guard<std::mutex> lk(battery_mutex_);
    return current_battery_;
}

void ChassisHal::setStatusCallback(std::function<void(const ChassisStatus&)> cb) {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    status_cb_ = std::move(cb);
}

void ChassisHal::setOdometryCallback(std::function<void(const OdometryData&)> cb) {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    odom_cb_ = std::move(cb);
}

void ChassisHal::setBatteryCallback(std::function<void(const BatteryData&)> cb) {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    battery_cb_ = std::move(cb);
}

// ─── Data reception ───────────────────────────────────────────────────────────

void ChassisHal::onDataReceived(const uint8_t* data, std::size_t len) {
    stream_decoder_.feed(data, len);
    while (auto frame = stream_decoder_.poll()) {
        onFrameDecoded(*frame);
    }
}

void ChassisHal::onFrameDecoded(const DecodedFrame& frame) {
    // Let the negotiator see all frames during handshake
    if (!negotiator_.isNegotiated()) {
        negotiator_.onFrameReceived(frame);
        return;
    }

    switch (frame.cmdId()) {
        case CmdId::CHASSIS_STATUS:
            dispatchStatusFrame(frame);
            break;
        case CmdId::ODOMETRY:
            dispatchOdometryFrame(frame);
            break;
        case CmdId::BATTERY_INFO:
            dispatchBatteryFrame(frame);
            break;
        case CmdId::ACK:
            // ACKs are informational; ignored here
            break;
        default:
            break;
    }
}

// ─── Uplink frame parsers ─────────────────────────────────────────────────────

void ChassisHal::dispatchStatusFrame(const DecodedFrame& frame) {
    // seq(4) + timestamp_us(8) + error_code(4) + error_msg_len(2) + error_msg(N)
    const auto& p = frame.payload;
    std::size_t off = 0;

    ChassisStatus st;
    const uint32_t seq = readU32LE(p, off);
    // Sequence numbers enable future out-of-order / drop detection
    (void)seq;
    st.timestamp_us = readU64LE(p, off);
    st.error_code   = readU32LE(p, off);
    const uint16_t msg_len = readU16LE(p, off);
    if (off + msg_len <= p.size()) {
        st.error_msg.assign(reinterpret_cast<const char*>(p.data() + off), msg_len);
        off += msg_len;
    }
    st.is_estop = (st.error_code & 0x8000U) != 0; // bit 15 set = emergency stop active

    {
        std::lock_guard<std::mutex> lk(status_mutex_);
        current_status_ = st;
    }

    std::function<void(const ChassisStatus&)> cb;
    {
        std::lock_guard<std::mutex> lk(cb_mutex_);
        cb = status_cb_;
    }
    if (cb) cb(st);
}

void ChassisHal::dispatchOdometryFrame(const DecodedFrame& frame) {
    // seq(4)+ts(8)+x(8)+y(8)+z(8)+qx(8)+qy(8)+qz(8)+qw(8)+vx(8)+vy(8)+omega(8)
    // + cov_pose(36*8) + cov_twist(36*8) = 668 bytes
    const auto& p = frame.payload;
    std::size_t off = 0;

    OdometryData od;
    const uint32_t seq = readU32LE(p, off);
    // Sequence numbers enable future out-of-order / drop detection
    (void)seq;
    od.timestamp_us = readU64LE(p, off);
    od.x     = readDouble(p, off);
    od.y     = readDouble(p, off);
    od.z     = readDouble(p, off);
    od.qx    = readDouble(p, off);
    od.qy    = readDouble(p, off);
    od.qz    = readDouble(p, off);
    od.qw    = readDouble(p, off);
    od.vx    = readDouble(p, off);
    od.vy    = readDouble(p, off);
    od.omega = readDouble(p, off);
    for (int i = 0; i < 36; ++i) od.cov_pose[i]  = readDouble(p, off);
    for (int i = 0; i < 36; ++i) od.cov_twist[i] = readDouble(p, off);

    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        current_odom_ = od;
    }

    std::function<void(const OdometryData&)> cb;
    {
        std::lock_guard<std::mutex> lk(cb_mutex_);
        cb = odom_cb_;
    }
    if (cb) cb(od);
}

void ChassisHal::dispatchBatteryFrame(const DecodedFrame& frame) {
    // voltage(8)+current(8)+percentage(8)+temperature(8)+is_charging(1)+num_cells(1)+cells(N*8)
    const auto& p = frame.payload;
    std::size_t off = 0;

    BatteryData bat;
    bat.voltage     = readDouble(p, off);
    bat.current     = readDouble(p, off);
    bat.percentage  = readDouble(p, off);
    bat.temperature = readDouble(p, off);
    bat.is_charging = (readU8(p, off) != 0);
    const uint8_t num_cells = readU8(p, off);
    bat.cell_voltages.resize(num_cells);
    for (uint8_t i = 0; i < num_cells; ++i) {
        bat.cell_voltages[i] = readDouble(p, off);
    }

    {
        std::lock_guard<std::mutex> lk(battery_mutex_);
        current_battery_ = bat;
    }

    std::function<void(const BatteryData&)> cb;
    {
        std::lock_guard<std::mutex> lk(cb_mutex_);
        cb = battery_cb_;
    }
    if (cb) cb(bat);
}

// ─── Reconnect loop ───────────────────────────────────────────────────────────

void ChassisHal::reconnectLoop() {
    long backoff_ms = 500;
    static constexpr long MAX_BACKOFF_MS = 30000;

    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (!running_.load()) break;

        if (transport_->isConnected() && negotiator_.isNegotiated()) {
            backoff_ms = 500;
            continue;
        }

        // Not connected — attempt reconnect
        connected_.store(false);

        if (!transport_->isConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            backoff_ms = std::min(backoff_ms * 2, MAX_BACKOFF_MS);

            if (!transport_->connect()) {
                continue;
            }
        }

        // Transport up — redo handshake
        negotiator_.reset();
        stream_decoder_.reset();

        if (!negotiator_.sendHandshake()) {
            transport_->disconnect();
            continue;
        }

        // Wait for negotiation
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(6);
        while (running_.load() && std::chrono::steady_clock::now() < deadline) {
            if (negotiator_.isNegotiated()) break;
            if (negotiator_.checkTimeout()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (negotiator_.isNegotiated()) {
            connected_.store(true);
            backoff_ms = 500;
        } else {
            transport_->disconnect();
        }
    }
}

} // namespace chassis
