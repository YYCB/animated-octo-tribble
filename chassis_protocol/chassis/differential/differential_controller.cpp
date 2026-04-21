#include "differential_controller.hpp"

#include <algorithm>

namespace chassis {

DifferentialController::DifferentialController(std::unique_ptr<ITransport> transport,
                                               uint32_t protocol_version)
    : ChassisHal(std::move(transport), protocol_version) {}

bool DifferentialController::sendVelocity(double vx, double vy, double omega) {
    // Differential drive: lateral velocity is always zero.
    vy = 0.0;
    vx    = std::max(-max_linear_vel_,  std::min(max_linear_vel_,  vx));
    omega = std::max(-max_angular_vel_, std::min(max_angular_vel_, omega));
    return ChassisHal::sendVelocity(vx, vy, omega);
}

ChassisInfo DifferentialController::getInfo() const {
    ChassisInfo info = ChassisHal::getInfo();
    info.chassis_type = ChassisType::DIFFERENTIAL;
    info.vendor       = "Generic";
    info.model        = "DifferentialDrive";
    return info;
}

void DifferentialController::setWheelbase(double meters) {
    wheelbase_ = meters;
}

void DifferentialController::setMaxLinearVel(double mps) {
    max_linear_vel_ = mps;
}

void DifferentialController::setMaxAngularVel(double rps) {
    max_angular_vel_ = rps;
}

} // namespace chassis
