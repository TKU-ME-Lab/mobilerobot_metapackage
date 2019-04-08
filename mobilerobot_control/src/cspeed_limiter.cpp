#include <algorithm>

#include <mobilerobot_control/cspeed_limiter.h>

template<typename T>
T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

namespace mecanum_wheel_controller
{
  CSpeedLimiter::CSpeedLimiter(bool has_velocity_limits,    bool has_acceleration_limits,
                              double min_velocity, double max_velocity,
                              double min_acceleration, double max_acceleration):
    m_has_velocity_limits(has_velocity_limits), m_has_accelation_limits(has_acceleration_limits),
    m_min_velocity(min_velocity), m_max_velocity(max_velocity),
    m_min_acceleration(min_acceleration), m_max_acceleration(m_max_acceleration)
  {
  }

  void CSpeedLimiter::limit(double &v, double v0, double dt)
  {
    limit_velocity(v);
    limit_acceleration(v, v0, dt);
  }

  void CSpeedLimiter::limit_velocity(double &v)
  {
    if (m_has_velocity_limits)
    {
      v = clamp(v, m_min_velocity, m_max_velocity);
    }
  }

  void CSpeedLimiter::limit_acceleration(double &v, double v0, double dt)
  {
    if (m_has_accelation_limits)
    {
      double dv = v - v0;
      const double dv_min = m_min_acceleration * dt;
      const double dv_max = m_max_acceleration * dt;

      dv = clamp(dv, dv_min, dv_max);
      v = v0 + dv;
    }
  }

} // mecanum_whell_controller