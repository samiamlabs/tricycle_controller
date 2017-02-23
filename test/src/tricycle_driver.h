#ifndef TRICYCLE_DRIVER_H
#define TRICYCLE_DRIVER_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

class Tricycle : public hardware_interface::RobotHW
{
public:
  Tricycle();
  ~Tricycle() {}

  void read();
  void write();

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.02);}

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface joint_pos_interface_;
  hardware_interface::VelocityJointInterface joint_vel_interface_;

  double steer_cmd_, steer_pos_, steer_vel_, steer_eff_;
  double motor_cmd_, motor_pos_, motor_vel_, motor_eff_;

};

#endif
