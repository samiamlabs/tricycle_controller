#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <urdf_vehicle_kinematic/urdf_vehicle_kinematic.h>
#include <geometry_msgs/Twist.h>
#include <tricycle_controller/speed_limiter.h>
#include <realtime_tools/realtime_buffer.h>

#include <tricycle_controller/wheel_kinematics.h>

namespace tricycle_controller
{

class TricycleController: public controller_interface::ControllerBase
{
public:
  TricycleController();
  ~TricycleController(){}

  virtual bool initRequest(hardware_interface::RobotHW *const robot_hw,
                            ros::NodeHandle& root_nh,
                            ros::NodeHandle &controller_nh,
                            std::set<std::string>& claimed_resources);

  bool init(hardware_interface::PositionJointInterface* hw_pos,
            hardware_interface::VelocityJointInterface *hw_vel,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh);

  std::string getHardwareInterfaceType() const
  {
    return "";
  }

  void starting(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

  void stopping(const ros::Time&);

private:

  void update_movement(const ros::Time& time, const ros::Duration& period);
  void update_odoemtry(const ros::Time& time, const ros::Duration& period);

  void cmdVelCallback(const geometry_msgs::TwistConstPtr& cmd_vel_msg);
  void setCmdVel(const geometry_msgs::TwistConstPtr& cmd_vel_msg);

  std::string name_;

  // Hardware handles:
  hardware_interface::JointHandle drive_wheel_joint_;
  hardware_interface::JointHandle drive_wheel_steer_joint_;

  double wheel_base_;
  double drive_wheel_radius_;
  bool wheel_is_reversed_;

  bool new_cmd_available_;

  WheelKinematics drive_wheel_kinematics_;

  struct Commands
  {
    double linear;
    double angular;
    ros::Time stamp;

    Commands() : linear(0.0), angular(0.0), stamp(0.0) {}
  };

  realtime_tools::RealtimeBuffer<Commands> command_;

  Commands command_struct_;

  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;
  Commands last1_cmd_;
  Commands last0_cmd_;

  SpeedLimiter limiter_steer_servo_;
  double last1_steer_angle_;
  double last0_steer_angle_;

  SpeedLimiter limiter_wheel_motor_;
  double last1_wheel_speed_;
  double last0_wheel_speed_;

  ros::Subscriber cmd_vel_sub_;

  double cmd_vel_timeout_;

};


PLUGINLIB_EXPORT_CLASS(tricycle_controller::TricycleController, controller_interface::ControllerBase);

}
