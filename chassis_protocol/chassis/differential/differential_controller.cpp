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

    if (mode_ == ControlMode::WHEEL_RPM) {
        // Inverse kinematics: convert body velocity to per-wheel angular velocity (rad/s).
        // Wheel order: [0] left, [1] right.
        const double half_wb = wheelbase_ / 2.0;
        const double inv_r   = 1.0 / wheel_radius_;
        const std::vector<double> wheels = {
            (vx - half_wb * omega) * inv_r,   // left
            (vx + half_wb * omega) * inv_r,   // right
        };
        return ChassisHal::sendWheelCmd(wheels);
    }

    return ChassisHal::sendVelocity(vx, vy, omega);
}

bool DifferentialController::sendWheelCmd(const std::vector<double>& wheel_rpms_rads) {
    return ChassisHal::sendWheelCmd(wheel_rpms_rads);
}

ChassisInfo DifferentialController::getInfo() const {
    ChassisInfo info = ChassisHal::getInfo();
    info.chassis_type = ChassisType::DIFFERENTIAL;
    info.vendor       = "Generic";
    info.model        = "DifferentialDrive";
    return info;
}

void DifferentialController::setControlMode(ControlMode mode) { mode_           = mode; }
void DifferentialController::setWheelbase(double meters)      { wheelbase_      = meters; }
void DifferentialController::setWheelRadius(double meters)    { wheel_radius_   = meters; }
void DifferentialController::setMaxLinearVel(double mps)      { max_linear_vel_ = mps; }
void DifferentialController::setMaxAngularVel(double rps)     { max_angular_vel_ = rps; }

} // namespace chassis
