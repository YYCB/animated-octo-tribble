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
    return ChassisHal::sendVelocity(vx, vy, omega);
}

ChassisInfo MecanumController::getInfo() const {
    ChassisInfo info = ChassisHal::getInfo();
    info.chassis_type = ChassisType::MECANUM;
    info.vendor       = "Generic";
    info.model        = "MecanumDrive";
    return info;
}

void MecanumController::setWheelRadius(double meters) { wheel_radius_ = meters; }
void MecanumController::setWheelbase(double meters)   { wheelbase_    = meters; }
void MecanumController::setTrack(double meters)       { track_        = meters; }
void MecanumController::setMaxLinearVel(double mps)   { max_linear_vel_  = mps; }
void MecanumController::setMaxAngularVel(double rps)  { max_angular_vel_ = rps; }

} // namespace chassis
