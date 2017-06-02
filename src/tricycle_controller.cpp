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
 
#include <tricycle_controller/tricycle_controller.h>

namespace tricycle_controller
{

TricycleController::TricycleController()
  :
  cmd_vel_timeout_(0.5),
  command_struct_(),
  last1_steer_angle_(0),
  last0_steer_angle_(0),
  last1_wheel_speed_(0),
  last0_wheel_speed_(0),
  wheel_base_(1.0),
  drive_wheel_radius_(0.4),
  wheel_is_reversed_(false),
  angle_tolerance_(0.3)
{
}


bool TricycleController::initRequest(hardware_interface::RobotHW *const robot_hw,
                                                  ros::NodeHandle& root_nh,
                                                  ros::NodeHandle &controller_nh,
                                                  std::set<std::string>& claimed_resources)
{
  ROS_INFO("Running initRequest woooop");
  if(state_ != CONSTRUCTED)
  {
    ROS_ERROR("The tricycle drive controller could not be created.");
    return false;
  }

  hardware_interface::PositionJointInterface *const pos_joint_hw = robot_hw->get<hardware_interface::PositionJointInterface>();
  hardware_interface::VelocityJointInterface *const vel_joint_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();

  if(pos_joint_hw == NULL)
  {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>().c_str());
      return false;
  }else if(vel_joint_hw == NULL)
  {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>().c_str());
      return false;
  }

  pos_joint_hw->clearClaims();
  vel_joint_hw->clearClaims();
  if(init(pos_joint_hw, vel_joint_hw, root_nh, controller_nh) == false)
  {
    ROS_ERROR("Failed to initialize the controller");
    return false;
  }

  claimed_resources.clear();

  const std::set<std::string> claims_pos = pos_joint_hw->getClaims();
  claimed_resources.insert(claims_pos.begin(), claims_pos.end());
  pos_joint_hw->clearClaims();

  const std::set<std::string> claims_vel = vel_joint_hw->getClaims();
  claimed_resources.insert(claims_vel.begin(), claims_vel.end());
  vel_joint_hw->clearClaims();

  state_ = INITIALIZED;
  return true;
}

bool TricycleController::init( hardware_interface::PositionJointInterface* hw_pos,
                                    hardware_interface::VelocityJointInterface *hw_vel,
                                    ros::NodeHandle& root_nh,
                                    ros::NodeHandle &controller_nh)
{
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");

  ROS_INFO("TricycleController namespace: %s", complete_ns.c_str());

  std::string drive_wheel_name, drive_wheel_steer_name;

  controller_nh.param<std::string>("drive_wheel", drive_wheel_name, "drive_wheel_joint");
  controller_nh.param<std::string>("drive_wheel_steer", drive_wheel_steer_name, "drive_wheel_steer_joint");

  // Velocity and acceleration limits:
  controller_nh.param("linear/x/has_velocity_limits"    , limiter_linear_.has_velocity_limits    , limiter_linear_.has_velocity_limits    );
  controller_nh.param("linear/x/has_acceleration_limits", limiter_linear_.has_acceleration_limits, limiter_linear_.has_acceleration_limits);
  controller_nh.param("linear/x/has_jerk_limits"        , limiter_linear_.has_jerk_limits        , limiter_linear_.has_jerk_limits        );
  controller_nh.param("linear/x/max_velocity"           , limiter_linear_.max_velocity           ,  limiter_linear_.max_velocity          );
  controller_nh.param("linear/x/min_velocity"           , limiter_linear_.min_velocity           , -limiter_linear_.max_velocity          );
  controller_nh.param("linear/x/max_acceleration"       , limiter_linear_.max_acceleration       ,  limiter_linear_.max_acceleration      );
  controller_nh.param("linear/x/min_acceleration"       , limiter_linear_.min_acceleration       , -limiter_linear_.max_acceleration      );
  controller_nh.param("linear/x/max_jerk"               , limiter_linear_.max_jerk               ,  limiter_linear_.max_jerk              );
  controller_nh.param("linear/x/min_jerk"               , limiter_linear_.min_jerk               , -limiter_linear_.max_jerk              );

  controller_nh.param("angular/z/has_velocity_limits"    , limiter_angular_.has_velocity_limits    , limiter_angular_.has_velocity_limits    );
  controller_nh.param("angular/z/has_acceleration_limits", limiter_angular_.has_acceleration_limits, limiter_angular_.has_acceleration_limits);
  controller_nh.param("angular/z/has_jerk_limits"        , limiter_angular_.has_jerk_limits        , limiter_angular_.has_jerk_limits        );
  controller_nh.param("angular/z/max_velocity"           , limiter_angular_.max_velocity           ,  limiter_angular_.max_velocity          );
  controller_nh.param("angular/z/min_velocity"           , limiter_angular_.min_velocity           , -limiter_angular_.max_velocity          );
  controller_nh.param("angular/z/max_acceleration"       , limiter_angular_.max_acceleration       ,  limiter_angular_.max_acceleration      );
  controller_nh.param("angular/z/min_acceleration"       , limiter_angular_.min_acceleration       , -limiter_angular_.max_acceleration      );
  controller_nh.param("angular/z/max_jerk"               , limiter_angular_.max_jerk               ,  limiter_angular_.max_jerk              );
  controller_nh.param("angular/z/min_jerk"               , limiter_angular_.min_jerk               , -limiter_angular_.max_jerk              );

  controller_nh.param("steer_servo/has_position_limits"     , limiter_steer_servo_.has_position_limits      , limiter_steer_servo_.has_position_limits      );
  controller_nh.param("steer_servo/has_velocity_limits"     , limiter_steer_servo_.has_velocity_limits      , limiter_steer_servo_.has_velocity_limits      );
  controller_nh.param("steer_servo/has_acceleration_limits" , limiter_steer_servo_.has_acceleration_limits  , limiter_steer_servo_.has_acceleration_limits  );
  controller_nh.param("steer_servo/has_jerk_limits"         , limiter_steer_servo_.has_jerk_limits          , limiter_steer_servo_.has_jerk_limits          );
  controller_nh.param("steer_servo/max_position"            , limiter_steer_servo_.max_position             ,  limiter_steer_servo_.max_position            );
  controller_nh.param("steer_servo/min_position"            , limiter_steer_servo_.min_position             , -limiter_steer_servo_.max_position            );
  controller_nh.param("steer_servo/max_velocity"            , limiter_steer_servo_.max_velocity             ,  limiter_steer_servo_.max_velocity            );
  controller_nh.param("steer_servo/min_velocity"            , limiter_steer_servo_.min_velocity             , -limiter_steer_servo_.max_velocity            );
  controller_nh.param("steer_servo/max_acceleration"        , limiter_steer_servo_.max_acceleration         ,  limiter_steer_servo_.max_acceleration        );
  controller_nh.param("steer_servo/min_acceleration"        , limiter_steer_servo_.min_acceleration         , -limiter_steer_servo_.max_acceleration        );
  controller_nh.param("steer_servo/max_jerk"                , limiter_steer_servo_.max_jerk                 ,  limiter_steer_servo_.max_jerk                );
  controller_nh.param("steer_servo/min_jerk"                , limiter_steer_servo_.min_jerk                 , -limiter_steer_servo_.max_jerk                );

  controller_nh.param("steer_servo/oscillation_damper_constant", limiter_steer_servo_.oscillation_damper_constant, limiter_steer_servo_.oscillation_damper_constant);
  controller_nh.param("steer_servo/min_damper_vel", limiter_steer_servo_.min_damper_vel, limiter_steer_servo_.min_damper_vel);
  controller_nh.param("steer_servo/angle_tolerance", angle_tolerance_, angle_tolerance_);

  controller_nh.param("wheel_motor/has_velocity_limits"    , limiter_wheel_motor_.has_velocity_limits    , limiter_wheel_motor_.has_velocity_limits    );
  controller_nh.param("wheel_motor/has_acceleration_limits", limiter_wheel_motor_.has_acceleration_limits, limiter_wheel_motor_.has_acceleration_limits);
  controller_nh.param("wheel_motor/has_jerk_limits"        , limiter_wheel_motor_.has_jerk_limits        , limiter_wheel_motor_.has_jerk_limits        );
  controller_nh.param("wheel_motor/max_angular_velocity"           , limiter_wheel_motor_.max_velocity           ,  limiter_wheel_motor_.max_velocity          );
  controller_nh.param("wheel_motor/min_angular_velocity"           , limiter_wheel_motor_.min_velocity           , -limiter_wheel_motor_.max_velocity          );
  controller_nh.param("wheel_motor/max_angular_acceleration"       , limiter_wheel_motor_.max_acceleration       ,  limiter_wheel_motor_.max_acceleration      );
  controller_nh.param("wheel_motor/min_angular_acceleration"       , limiter_wheel_motor_.min_acceleration       , -limiter_wheel_motor_.max_acceleration      );
  controller_nh.param("wheel_motor/max_jerk"               , limiter_wheel_motor_.max_jerk               ,  limiter_wheel_motor_.max_jerk              );
  controller_nh.param("wheel_motor/min_jerk"               , limiter_wheel_motor_.min_jerk               , -limiter_wheel_motor_.max_jerk              );

  ROS_INFO("max_trans_vel: %f, max_trans_rot: %f, max_acc_trans: %f, max_acc_rot: %f", limiter_linear_.max_velocity, limiter_angular_.max_velocity, limiter_linear_.max_acceleration, limiter_angular_.max_acceleration);
  ROS_INFO("Steer servo limits: vel: %f, acc: %f, jerk: %f", limiter_steer_servo_.max_velocity, limiter_steer_servo_.max_acceleration, limiter_steer_servo_.max_jerk);

  urdf_vehicle_kinematic::UrdfVehicleKinematic uvk(root_nh, "base_footprint");
  if(!uvk.getDistanceBetweenJointsX(drive_wheel_name, "base_footprint_joint", wheel_base_))
  {
    return false;
  }


  double wheel_rotation;
  if(!uvk.getRotationBetweenJoints(drive_wheel_name, "base_footprint_joint", wheel_rotation))
  {
    return false;
  }

  ROS_INFO("Wheel rotation is: %f", wheel_rotation);
  if(fabs(wheel_rotation) < 0.01)
  {
    ROS_INFO("Setting wheel direction to not revesed");
    wheel_is_reversed_ = false;
  }else if(fabs(wheel_rotation - M_PI) < 0.01)
  {
    ROS_INFO("Setting wheel direction to reversed");
    wheel_is_reversed_ = true;
  }else
  {
    ROS_INFO_STREAM_NAMED(name_,
                          "Only wheels that are mounted with center forwards "
                          << "or backwards are supported at this time. Aborting!");
  }

  if(!uvk.getJointRadius(drive_wheel_name, drive_wheel_radius_))
  {
    return false;
  }

  ROS_INFO("Wheel base set to: %f, and radius to: %f", wheel_base_, drive_wheel_radius_);
  drive_wheel_kinematics_.setMountPose(wheel_base_, 0);
  drive_wheel_kinematics_.setRadius(drive_wheel_radius_);

  ROS_INFO_STREAM_NAMED(name_,
                        "Adding drive wheel with joint name: " << drive_wheel_name
                        << " and steer with joint name: " << drive_wheel_steer_name);

  drive_wheel_joint_ = hw_vel->getHandle(drive_wheel_name);
  drive_wheel_steer_joint_ = hw_pos->getHandle(drive_wheel_steer_name);

  cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>("command", 1, &TricycleController::cmdVelCallback, this);

  return true;
}

void TricycleController::cmdVelCallback(const geometry_msgs::TwistConstPtr& cmd_vel_msg)
{
  if(isRunning())
  {
    setCmdVel(cmd_vel_msg);
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
  }
}

void TricycleController::setCmdVel(const geometry_msgs::TwistConstPtr& cmd_vel_msg)
{
  command_struct_.linear = cmd_vel_msg->linear.x;
  command_struct_.angular = cmd_vel_msg->angular.z;
  command_struct_.stamp = ros::Time::now();

  command_.writeFromNonRT (command_struct_);
}

void TricycleController::starting(const ros::Time& time)
{

}

void TricycleController::update(const ros::Time& time, const ros::Duration& period)
{
  update_movement(time, period);
}

void TricycleController::update_movement(const ros::Time& time, const ros::Duration& period)
{
  Commands current_command = *(command_.readFromRT());
  const double dt = (time - current_command.stamp).toSec();

  // Stop if time between cmd_vel messages is longer than set timeout
  if(dt > cmd_vel_timeout_)
  {
    current_command.linear = 0.0;
    current_command.angular = 0.0;
  }

  const double cmd_dt(period.toSec());

  //FIXME: prevent oscillations
  if(current_command.linear != 0)
  {
    limiter_linear_.limit(current_command.linear, last0_cmd_.linear, last1_cmd_.linear, cmd_dt);
  }

  if(current_command.angular != 0)
  {
    limiter_angular_.limit(current_command.angular, last0_cmd_.angular, last1_cmd_.angular, cmd_dt);
  }

  last1_cmd_ = last0_cmd_;
  last0_cmd_ = current_command;

  drive_wheel_kinematics_.setCommand(current_command.linear, current_command.angular);
  drive_wheel_kinematics_.update();

  double desired_steer_angle = drive_wheel_kinematics_.getSteer();
  double steer_angle_state = drive_wheel_steer_joint_.getPosition();

  //FIXME: do this in wheel_kinematics instaid...
  if(wheel_is_reversed_)
  {
    desired_steer_angle = -desired_steer_angle;
  }

  double filtered_steer_angle = desired_steer_angle;
  limiter_steer_servo_.limit(filtered_steer_angle, cmd_dt);

  last1_steer_angle_ = last0_steer_angle_;
  last0_steer_angle_ = filtered_steer_angle;

  double desired_angular_velocity = drive_wheel_kinematics_.getAngularVelocity();

  // ROS_INFO("desired: %f, state: %f, fabs: %f", desired_steer_angle, steer_angle_state, fabs(steer_angle_state - desired_steer_angle) );
  if(fabs(steer_angle_state - desired_steer_angle) > angle_tolerance_)
  {
    desired_angular_velocity = 0;
  }

  double filtered_angular_velocity = desired_angular_velocity;
  if(desired_angular_velocity != 0)
  {
    limiter_wheel_motor_.limit(filtered_angular_velocity, last0_wheel_speed_, last1_wheel_speed_, cmd_dt);
  }

  last1_wheel_speed_ = last0_wheel_speed_;
  last0_wheel_speed_ = filtered_angular_velocity;

  // ROS_INFO("Applied wheel speed: %f", filtered_angular_velocity);
  // ROS_INFO("Applied steer angle: %f, desired: %f", filtered_steer_angle, desired_steer_angle);
  // Update joints
  drive_wheel_joint_.setCommand(filtered_angular_velocity);
  drive_wheel_steer_joint_.setCommand(filtered_steer_angle);

}

void TricycleController::update_odoemtry(const ros::Time& time, const ros::Duration& period)
{
  //TODO: implement
}

void TricycleController::stopping(const ros::Time&)
{
  drive_wheel_joint_.setCommand(0);
}

}
