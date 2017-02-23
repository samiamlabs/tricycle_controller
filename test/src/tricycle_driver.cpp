#include "tricycle_driver.h"

Tricycle::Tricycle()
  :
  steer_cmd_(0),
  steer_pos_(0),
  steer_vel_(0),
  steer_eff_(0),
  motor_cmd_(0),
  motor_pos_(0),
  motor_vel_(0),
  motor_eff_(0)
{
  hardware_interface::JointStateHandle steer_state_handle("front_steering_joint",
                                                      &steer_pos_,
                                                      &steer_vel_,
                                                      &steer_eff_);
  joint_state_interface_.registerHandle(steer_state_handle);

  hardware_interface::JointStateHandle motor_state_handle("front_wheel",
                                                      &motor_pos_,
                                                      &motor_vel_,
                                                      &motor_eff_);
  joint_state_interface_.registerHandle(motor_state_handle);

  hardware_interface::JointHandle steer_handle(joint_state_interface_.getHandle("front_steering_joint"), &steer_cmd_);
  joint_pos_interface_.registerHandle(steer_handle);

  hardware_interface::JointHandle motor_handle(joint_state_interface_.getHandle("front_wheel"), &motor_cmd_);
  joint_vel_interface_.registerHandle(motor_handle);

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_pos_interface_);
  registerInterface(&joint_vel_interface_);
}

void Tricycle::read()
{
  //TODO: read from encoder or similar here

  // steer_pos_ = steer_cmd_; // loopback
  // motor_vel_ = motor_cmd_; // loopback
}

void Tricycle::write()
{
  // ROS_INFO("Writing steer position: %f, and motor_speed: %f", steer_cmd_, motor_cmd_);
}
