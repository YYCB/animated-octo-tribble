#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace chassis {

enum class ChassisType { DIFFERENTIAL, MECANUM, OMNI, UNKNOWN };
enum class LightMode { OFF, SOLID, BLINK, BREATH };

// Selects which command frame sendVelocity() emits.
//   BODY_VELOCITY  → VELOCITY_CMD (0x10): send (vx, vy, omega); MCU decomposes to wheels.
//   WHEEL_RPM      → WHEEL_CMD   (0x13): host decomposes to per-wheel rad/s; MCU only runs PID.
enum class ControlMode { BODY_VELOCITY, WHEEL_RPM };

struct ChassisInfo {
    std::string  vendor;
    std::string  model;
    ChassisType  chassis_type{ChassisType::UNKNOWN};
    std::string  firmware_version;
    uint32_t     protocol_version{0};
};

struct ChassisStatus {
    uint64_t    timestamp_us{0};
    uint32_t    error_code{0};
    std::string error_msg;
    bool        is_estop{false};
};

struct OdometryData {
    uint64_t timestamp_us{0};
    double   x{0}, y{0}, z{0};
    double   qx{0}, qy{0}, qz{0}, qw{1.0};
    double   vx{0}, vy{0}, omega{0};
    std::array<double, 36> cov_pose{};
    std::array<double, 36> cov_twist{};
};

struct BatteryData {
    double               voltage{0};
    double               current{0};
    double               percentage{0};
    double               temperature{0};
    bool                 is_charging{false};
    std::vector<double>  cell_voltages;
};

class IChassisController {
public:
    virtual ~IChassisController() = default;

    virtual bool sendVelocity(double vx, double vy, double omega) = 0;
    virtual bool sendLightCmd(LightMode mode, uint8_t r, uint8_t g, uint8_t b, double blink_hz) = 0;
    virtual bool emergencyStop(const std::string& reason) = 0;

    // Send per-wheel target velocities (rad/s).
    // wheel_rpms_rads must contain exactly as many elements as the chassis has wheels.
    // Wheel order is controller-specific (e.g. Mecanum: FL, FR, RL, RR; Diff: L, R).
    virtual bool sendWheelCmd(const std::vector<double>& wheel_rpms_rads) {
        (void)wheel_rpms_rads;
        return false;
    }

    virtual ChassisStatus  getStatus()      const = 0;
    virtual ChassisInfo    getInfo()        const = 0;
    virtual OdometryData   getOdometry()    const = 0;
    virtual BatteryData    getBatteryInfo() const = 0;

    virtual void setStatusCallback(std::function<void(const ChassisStatus&)> cb) = 0;
    virtual void setOdometryCallback(std::function<void(const OdometryData&)> cb) = 0;
    virtual void setBatteryCallback(std::function<void(const BatteryData&)> cb) = 0;
};

} // namespace chassis
