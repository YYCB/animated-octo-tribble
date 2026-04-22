#include "mecanum_controller.hpp"

#include <algorithm>

namespace chassis {

MecanumController::MecanumController(std::unique_ptr<ITransport> transport,
                                     uint32_t protocol_version)
    : ChassisHal(std::move(transport), protocol_version) {}

bool MecanumController::sendVelocity(double vx, double vy, double omega) {
    vx    = std::max(-max_linear_vel_,  std::min(max_linear_vel_,  vx));
    vy    = std::max(-max_linear_vel_,  std::min(max_linear_vel_,  vy));
    omega = std::max(-max_angular_vel_, std::min(max_angular_vel_, omega));

    if (mode_ == ControlMode::WHEEL_RPM) {
        // Standard mecanum inverse kinematics.
        // Wheel order: [0] FL, [1] FR, [2] RL, [3] RR.
        // l = half wheelbase (front-rear), w = half track (left-right).
        const double l     = wheelbase_ / 2.0;
        const double w     = track_     / 2.0;
        const double inv_r = 1.0 / wheel_radius_;
        const std::vector<double> wheels = {
            (vx - vy - (l + w) * omega) * inv_r,   // FL
            (vx + vy + (l + w) * omega) * inv_r,   // FR
            (vx + vy - (l + w) * omega) * inv_r,   // RL
            (vx - vy + (l + w) * omega) * inv_r,   // RR
        };
        return ChassisHal::sendWheelCmd(wheels);
    }

    return ChassisHal::sendVelocity(vx, vy, omega);
}

bool MecanumController::sendWheelCmd(const std::vector<double>& wheel_rpms_rads) {
    return ChassisHal::sendWheelCmd(wheel_rpms_rads);
}

ChassisInfo MecanumController::getInfo() const {
    ChassisInfo info = ChassisHal::getInfo();
    info.chassis_type = ChassisType::MECANUM;
    info.vendor       = "Generic";
    info.model        = "MecanumDrive";
    return info;
}

void MecanumController::setControlMode(ControlMode mode)    { mode_           = mode; }
void MecanumController::setWheelRadius(double meters)       { wheel_radius_   = meters; }
void MecanumController::setWheelbase(double meters)         { wheelbase_      = meters; }
void MecanumController::setTrack(double meters)             { track_          = meters; }
void MecanumController::setMaxLinearVel(double mps)         { max_linear_vel_ = mps; }
void MecanumController::setMaxAngularVel(double rps)        { max_angular_vel_ = rps; }

} // namespace chassis
