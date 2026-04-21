#pragma once

#include "../chassis_hal.hpp"

namespace chassis {

class DifferentialController final : public ChassisHal {
public:
    explicit DifferentialController(std::unique_ptr<ITransport> transport,
                                    uint32_t protocol_version = 1);

    bool        sendVelocity(double vx, double vy, double omega) override;
    ChassisInfo getInfo() const override;

    void setWheelbase(double meters);
    void setMaxLinearVel(double mps);
    void setMaxAngularVel(double rps);

private:
    double wheelbase_{0.5};
    double max_linear_vel_{1.5};
    double max_angular_vel_{3.14};
};

} // namespace chassis
