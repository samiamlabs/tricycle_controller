/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Toyota Material Handling Sweden
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
 *   * Neither the name of the copyright holder nor the names of its
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

#include <math.h>
#include <tricycle_controller/wheel_kinematics.h>

int getIndexOfSmallestElement(double arr[], int size)
{
  int smallestIndex = 0;
  for (int index = smallestIndex; index < size; index++)
    {
      if (fabs(arr[smallestIndex]) > fabs(arr[index]))
    {
      smallestIndex = index;
    }
  }
  return smallestIndex;
}


WheelKinematics::WheelKinematics():
  pose_x_(0.4),
  pose_y_(0),
  radius_(0.115),
  max_steer_(175),
  STEER_ANGLE_TOLERANCE(0.3),
  MINIMUM_TRANSLATIONAL_VELOCITY(0.01),
  USE_ANGLE_TOLERANCE(false),
  USE_LIMITED_STEER_ANGLE(false),
  motor_rotational_velocity_(0),
  servo_steer_angle_(0),
  motor_direction_(0),
  desired_wheel_mount_velocity_x_(0),
  desired_wheel_mount_velocity_y_(0),
  servo_steer_angle_state_(0),
  motor_rotational_velocity_state_(0)
{
}

void WheelKinematics::setMountPose(double x, double y)
{
  pose_x_ = x;
  pose_y_ = y;
}

void WheelKinematics::setRadius(double radius)
{
  radius_ = radius;
}

void WheelKinematics::setCommand(double linear_x, double angular_z)
{
  double linear_y = 0;
  updateDesiredWheelMountMovement(linear_x, linear_y, angular_z);
}

void WheelKinematics::setSteerAngleState(double angle)
{
  servo_steer_angle_state_ = angle;
}

void WheelKinematics::setAngularVelocityState(double vel)
{
  motor_rotational_velocity_state_ = vel;
}

void WheelKinematics::update()
{
  // the following functions make use of desired
  // wheel mount velocitys set by cmdVelCallback
  updateServoAngleAndDirection();
  updateMotorRotationalVelocity();
}

double WheelKinematics::getSteer()
{
  return servo_steer_angle_;
}

double WheelKinematics::getAngularVelocity()
{
  return motor_rotational_velocity_;
}

void WheelKinematics::updateServoAngleAndDirection()
{
  double translational_velocity = sqrt( pow(desired_wheel_mount_velocity_x_, 2) + pow(desired_wheel_mount_velocity_y_, 2) );
  if( fabs(translational_velocity) > MINIMUM_TRANSLATIONAL_VELOCITY)
  {
    double unimproved_servo_steer_angle = calculateUnimprovedServoAngle();
    setImprovedServoAngleAndMotorDirection(unimproved_servo_steer_angle);
  }
}

void WheelKinematics::updateMotorRotationalVelocity()
{
  // ROS_INFO("Desired angle: %f, State: %f", servo_steer_angle_, servo_steer_angle_state_);
  if(USE_ANGLE_TOLERANCE && (fabs(servo_steer_angle_ - servo_steer_angle_state_) > STEER_ANGLE_TOLERANCE) )
  {
    motor_rotational_velocity_ = 0;
  }
  else
  {
    double translational_velocity = sqrt( pow(desired_wheel_mount_velocity_x_, 2) + pow(desired_wheel_mount_velocity_y_, 2) );
    motor_rotational_velocity_ = (motor_direction_*translational_velocity)/radius_;
  }
}


void WheelKinematics::updateDesiredWheelMountMovement(double origo_velocity_x, double origo_velocity_y, double origo_rotation)
{
  double circular_motion_phi = atan2(pose_y_, pose_x_);
  double circular_motion_r = sqrt( pow(pose_x_, 2) + pow(pose_y_, 2) );

  // rigid body implies the following if origo_rotation is zero
  double desired_wheel_mount_velocity_x = origo_velocity_x;
  double desired_wheel_mount_velocity_y = origo_velocity_y;

  // add movement caused by rotation around origo
  if(fabs(origo_rotation) > 0.01)
  {
    double total_velocity_from_rotation = origo_rotation*circular_motion_r;
    // TODO: test if the following works for y offset not zero
    desired_wheel_mount_velocity_x += total_velocity_from_rotation*cos(circular_motion_phi+M_PI/2);
    desired_wheel_mount_velocity_y += total_velocity_from_rotation*sin(circular_motion_phi+M_PI/2);
  }

  desired_wheel_mount_velocity_x_ = desired_wheel_mount_velocity_x;
  desired_wheel_mount_velocity_y_ = desired_wheel_mount_velocity_y;
}

double WheelKinematics::calculateUnimprovedServoAngle()
{
  return -atan2(desired_wheel_mount_velocity_y_, desired_wheel_mount_velocity_x_);
}

void WheelKinematics::setImprovedServoAngleAndMotorDirection(double unimproved_servo_steer_angle)
{
  double angle_reverse_left = unimproved_servo_steer_angle + M_PI;
  double angle_reverse_right = unimproved_servo_steer_angle - M_PI;

  double angle_mod = fmod(unimproved_servo_steer_angle + 2*M_PI, 2*M_PI);
  double angle_reverse_left_mod = fmod(angle_reverse_left + 2*M_PI, 2*M_PI);
  double angle_reverse_right_mod = fmod(angle_reverse_right + 2*M_PI, 2*M_PI);

  double err_default = servo_steer_angle_state_ - unimproved_servo_steer_angle;
  double err_rl = servo_steer_angle_state_ - angle_reverse_left;
  double err_rr = servo_steer_angle_state_ - angle_reverse_right;
  double err_mod = servo_steer_angle_state_ - angle_mod;

  double err_rl_m = servo_steer_angle_state_ - angle_reverse_left_mod;
  double err_rr_m = servo_steer_angle_state_ - angle_reverse_right_mod;

  double angles[6] = {unimproved_servo_steer_angle,
                      angle_reverse_left,
                      angle_reverse_right,
                      angle_mod,
                      angle_reverse_left_mod,
                      angle_reverse_right_mod};

  double angle_errors[6] =    {err_default,
                              err_rl,
                              err_rr,
                              err_mod,
                              err_rl_m,
                              err_rr_m};

  int motor_directions[6] = {1,
                            -1,
                            -1,
                             1,
                            -1,
                            -1 };

  for(int i = 0; i < 6; i++)
  {
    if (fabs(angles[i]) > M_PI || fabs(angles[i]) > max_steer_ )
    {
      angle_errors[i] = 999;
    }
  }

  int index_of_closest_angle = getIndexOfSmallestElement(angle_errors, 6);

  servo_steer_angle_ = -angles[index_of_closest_angle];
  motor_direction_ = motor_directions[index_of_closest_angle];
}
