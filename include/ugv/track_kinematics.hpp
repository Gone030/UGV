#pragma once

#include <optional>

namespace ugv {

struct TrackVelocityTargets {
  double left_rad_s{};
  double right_rad_s{};
};

class TrackKinematics {
 public:
  TrackKinematics(double track_width_m, double drive_sprocket_radius_m);

  bool configured() const;
  std::optional<TrackVelocityTargets> calculate(double linear_x_m_s,
                                                double angular_z_rad_s) const;

 private:
  double track_width_m_{};
  double drive_sprocket_radius_m_{};
};

}  // namespace ugv
