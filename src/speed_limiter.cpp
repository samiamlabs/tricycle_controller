/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 */

#include <algorithm>

#include <tricycle_controller/speed_limiter.h>

template<typename T>
T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

namespace tricycle_controller
{

  SpeedLimiter::SpeedLimiter(
    bool has_velocity_limits,
    bool has_acceleration_limits,
    bool has_jerk_limits,
    double min_velocity,
    double max_velocity,
    double min_acceleration,
    double max_acceleration,
    double min_jerk,
    double max_jerk
  )
  : has_velocity_limits(has_velocity_limits)
  , has_acceleration_limits(has_acceleration_limits)
  , has_jerk_limits(has_jerk_limits)
  , min_velocity(min_velocity)
  , max_velocity(max_velocity)
  , min_acceleration(min_acceleration)
  , max_acceleration(max_acceleration)
  , min_jerk(min_jerk)
  , max_jerk(max_jerk)
  {
  }

  double SpeedLimiter::limit(double& v, double v0, double v1, double dt)
  {
    const double tmp = v;

    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_velocity(double& v)
  {
    const double tmp = v;

    if (has_velocity_limits)
    {
      v = clamp(v, min_velocity, max_velocity);
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_acceleration(double& v, double v0, double dt)
  {
    const double tmp = v;

    if (has_acceleration_limits)
    {
      const double dv_min = min_acceleration * dt;
      const double dv_max = max_acceleration * dt;

      const double dv = clamp(v - v0, dv_min, dv_max);

      v = v0 + dv;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_jerk(double& v, double v0, double v1, double dt)
  {
    const double tmp = v;

    if (has_jerk_limits)
    {
      const double dv  = v  - v0;
      const double dv0 = v0 - v1;

      const double dt2 = 2. * dt * dt;

      const double da_min = min_jerk * dt2;
      const double da_max = max_jerk * dt2;

      const double da = clamp(dv - dv0, da_min, da_max);

      v = v0 + dv0 + da;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  PositionLimiter::PositionLimiter(
    bool has_position_limit,
    bool has_velocity_limits,
    bool has_acceleration_limits,
    bool has_jerk_limits,
    double min_position,
    double max_position,
    double min_velocity,
    double max_velocity,
    double min_acceleration,
    double max_acceleration,
    double min_jerk,
    double max_jerk,
    double oscillation_damper_constant,
    double min_damper_vel
  )
  : has_position_limits(has_position_limits)
  , has_velocity_limits(has_velocity_limits)
  , has_acceleration_limits(has_acceleration_limits)
  , has_jerk_limits(has_jerk_limits)
  , min_position(min_position)
  , max_position(max_position)
  , min_velocity(min_velocity)
  , max_velocity(max_velocity)
  , min_acceleration(min_acceleration)
  , max_acceleration(max_acceleration)
  , min_jerk(min_jerk)
  , max_jerk(max_jerk)
  , oscillation_damper_constant(oscillation_damper_constant)
  , min_damper_vel(min_damper_vel)
  {
  }

  double PositionLimiter::limit(double& position, double dt)
  {
    update_state(position, dt);

    limit_jerk();
    limit_acceleration();
    limit_velocity();
    limit_position();

    oscillation_damper();

    update_last_state();

    position = current_position_;

    return 1.0;  // tmp != 0.0 ? v / tmp : 1.0;
  }

  void PositionLimiter::update_state(double position, double dt)
  {
    dt_ = dt;
    target_position_ = position;
    current_position_ = position;
    current_velocity_ = (current_position_ - last_position_)/dt;
    current_acceleration_ = (current_velocity_ - last_velocity_)/dt;
    current_jerk_ = (current_acceleration_ - last_acceleration_)/dt;
  }

  void PositionLimiter::limit_position()
  {
    if (has_position_limits)
    {
      current_position_ = clamp(current_position_, min_position, max_position);
    }
  }

  void PositionLimiter::limit_velocity()
  {
    if (has_velocity_limits)
    {
      current_velocity_ = clamp(current_velocity_, min_velocity, max_velocity);
      current_position_ = last_position_ + current_velocity_*dt_;
    }
  }

  void PositionLimiter::limit_acceleration()
  {
    if (has_acceleration_limits)
    {
      current_acceleration_ = clamp(current_acceleration_, min_acceleration, max_acceleration);
      current_velocity_ = last_velocity_ + current_acceleration_*dt_;
      current_position_ = last_position_ + current_velocity_*dt_;
    }
  }

  void PositionLimiter::limit_jerk()
  {
    if (has_jerk_limits)
    {
      current_jerk_ = clamp(current_jerk_, min_jerk, max_jerk);
      current_acceleration_ = last_acceleration_ + current_jerk_*dt_;
      current_velocity_ = last_velocity_ + current_acceleration_*dt_;
      current_position_ = last_position_ + current_velocity_*dt_;
    }
  }

  void PositionLimiter::oscillation_damper()
  {
    double oscillation_damped_velocity = min_damper_vel + pow(fabs(oscillation_damper_constant*(target_position_-current_position_)*current_velocity_), 2);
    current_velocity_ = clamp(current_velocity_, -oscillation_damped_velocity, oscillation_damped_velocity);
  }

  void PositionLimiter::update_last_state()
  {
    last_position_ = current_position_;
    last_velocity_ = current_velocity_;
    last_acceleration_ = current_acceleration_;
    last_jerk_ = current_jerk_;
  }

} // namespace tricycle_controller
