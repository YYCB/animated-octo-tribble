#pragma once

#include "../chassis_hal.hpp"

namespace chassis {

class MecanumController final : public ChassisHal {
public:
    explicit MecanumController(std::unique_ptr<ITransport> transport,
                               uint32_t protocol_version = 1);

    bool        sendVelocity(double vx, double vy, double omega) override;
    bool        sendWheelCmd(const std::vector<double>& wheel_rpms_rads) override;
    ChassisInfo getInfo() const override;

    // Select whether sendVelocity() transmits body velocity (VELOCITY_CMD, MCU does IK)
    // or computes per-wheel rad/s on the host and transmits WHEEL_CMD.
    // Wheel order for WHEEL_RPM mode: [0] FL, [1] FR, [2] RL, [3] RR.
    void setControlMode(ControlMode mode);

    void setWheelRadius(double meters);
    void setWheelbase(double meters);
    void setTrack(double meters);
    void setMaxLinearVel(double mps);
    void setMaxAngularVel(double rps);

private:
    ControlMode mode_{ControlMode::BODY_VELOCITY};
    double wheel_radius_{0.076};
    double wheelbase_{0.4};
    double track_{0.4};
    double max_linear_vel_{1.5};
    double max_angular_vel_{3.14};
};

} // namespace chassis
