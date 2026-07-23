#include "ugv/track_kinematics.hpp"

#include <cmath>

namespace ugv {

TrackKinematics::TrackKinematics(double track_width_m, double drive_sprocket_radius_m)
    : track_width_m_(track_width_m),
      drive_sprocket_radius_m_(drive_sprocket_radius_m) {}

bool TrackKinematics::configured() const {
  return std::isfinite(track_width_m_) && std::isfinite(drive_sprocket_radius_m_) &&
         track_width_m_ > 0.0 && drive_sprocket_radius_m_ > 0.0;
}

std::optional<TrackVelocityTargets> TrackKinematics::calculate(
    double linear_x_m_s, double angular_z_rad_s) const {
  if (!configured() || !std::isfinite(linear_x_m_s) ||
      !std::isfinite(angular_z_rad_s)) {
    return std::nullopt;
  }

  const double half_track = track_width_m_ * 0.5;
  const double left_linear_m_s = linear_x_m_s - angular_z_rad_s * half_track;
  const double right_linear_m_s = linear_x_m_s + angular_z_rad_s * half_track;
  return TrackVelocityTargets{
      left_linear_m_s / drive_sprocket_radius_m_,
      right_linear_m_s / drive_sprocket_radius_m_};
}

}  // namespace ugv
