#ifndef WHEEL_KINEMATICS
#define WHEEL_KINEMATICS

//FIXME: remove ros
#include "ros/ros.h"

class WheelKinematics
{
public:
  WheelKinematics();

  void setMountPose(double x, double y);
  void setRadius(double radius);

  void setCommand(double linear_x, double angular_z);
  void setSteerAngleState(double angle);
  void setAngularVelocityState(double vel);

  void update();

  double getSteer();
  double getAngularVelocity();


private:

  double STEER_ANGLE_TOLERANCE;
  double MINIMUM_TRANSLATIONAL_VELOCITY;
  bool USE_ANGLE_TOLERANCE;
  bool USE_LIMITED_STEER_ANGLE;

  double pose_x_, pose_y_;
  double radius_;
  double max_steer_;

  double motor_rotational_velocity_, servo_steer_angle_;
  int motor_direction_;

  double desired_wheel_mount_velocity_x_;
  double desired_wheel_mount_velocity_y_;

  double servo_steer_angle_state_, motor_rotational_velocity_state_;

  void updateDesiredWheelMountMovement(double origo_velocity_x, double origo_velocity_y, double origo_rotation);
  void updateServoAngleAndDirection();
  void updateMotorRotationalVelocity();
  double calculateUnimprovedServoAngle();
  void setImprovedServoAngleAndMotorDirection(double unimproved_servo_steer_angle);

};
#endif
