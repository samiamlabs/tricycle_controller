// License: BSD
// https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <tricycle_controller/odometry.h>
#include <tricycle_controller/speed_limiter.h>

#include <tricycle_controller/wheel_kinematics.h>

namespace tricycle_controller {

class TricycleController
    : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                                   hardware_interface::PositionJointInterface>
{
public:
  /**
   * This class makes some assumptions on the model of the robot:
   *  - the steer joint is not rotated
   *  - the steer joint is centered in the y-direction
   *  - the drive wheel is centered in the y-direction (no y-offset like
   * forklifts sometimes have) Additional assumptions to not duplicate
   * information readily available in the URDF:
   *  - a wheel collision geometry is a cylinder in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie
   * within the contact patch
   */

  TricycleController();

  /**
   * \brief Initialize controller
   * \param robot_hw      Velocity and position joint interface for the drive
   * wheel \param root_nh       Node handle at root namespace \param
   * controller_nh Node handle inside the controller namespace
   */
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh);

  /**
   * \brief Updates controller, i.e. computes the odometry and sets the new
   * velocity commands \param time   Current time \param period Time since the
   * last called to update
   */
  void update(const ros::Time &time, const ros::Duration &period);

  /**
   * \brief Starts controller
   * \param time Current time
   */
  void starting(const ros::Time &time);

  /**
   * \brief Stops controller
   * \param time Current time
   */
  void stopping(const ros::Time &);

private:
  std::string name_;

  /// Hardware handles:
  hardware_interface::JointHandle drive_wheel_joint_;
  hardware_interface::JointHandle steer_joint_;

  WheelKinematics drive_wheel_kinematics_;

  /// Velocity command related
  struct Command {
    double lin;
    double ang;
    ros::Time stamp;

    Command() : lin(0.0), ang(0.0), stamp(0.0) {}
  };

  realtime_tools::RealtimeBuffer<Command> command_;
  Command command_struct_;
  ros::Subscriber sub_command_;

  /// Odometry related:
  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
  Odometry odometry_;

  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;
  bool open_loop_;

  /// Wheel base (distance between front and rear wheel):
  double wheel_base_;

  double wheel_radius_;

  /// Timeout to consider cmd_vel commands old:
  double cmd_vel_timeout_;

  /// Whether to allow multiple publishers on cmd_vel topic or not:
  bool allow_multiple_cmd_vel_publishers_;

  /// If the diffrence between desired steer angle its joint state is larger
  /// than angle_tolerance, stop wheel from turning
  double angle_tolerance_;

  /// Frame to use for the robot base:
  std::string base_frame_id_;

  /// Frame to use for odometry and odom tf:
  std::string odom_frame_id_;

  /// Whether to publish odometry to tf or not:
  bool enable_odom_tf_;

  /// Speed limiters:
  Command last1_cmd_;
  Command last0_cmd_;
  SpeedLimiter limiter_lin_;
  SpeedLimiter limiter_ang_;

private:
  /**
   * \brief Update and publish odometry
   * \param time   Current time
   */
  void updateOdometry(const ros::Time &time);
  /**
   * \brief Compute and publish command
   * \param time   Current time
   * \param period Time since the last called to update
   */
  void updateCommand(const ros::Time &time, const ros::Duration &period);

  /**
   * \brief Brakes the wheels, i.e. sets the velocity to 0
   */
  void brake();

  /**
   * \brief Velocity command callback
   * \param command Velocity command message (twist)
   */
  void cmdVelCallback(const geometry_msgs::Twist &command);

  /**
    * \brief Get the wheel names from a wheel param
    * \param [in]  controller_nh Controller node handler
    * \param [in]  wheel_param   Param name
    * \param [out] wheel_names   Vector with the whel names
    * \return true if the wheel_param is available and the wheel_names are
    *        retrieved successfully from the param server; false otherwise
    */
   bool getJointName(ros::NodeHandle& controller_nh,
                      const std::string& wheel_param,
                      std::string& wheel_name);

  /**
   * \brief Sets the odometry publishing fields
   * \param root_nh Root node handle
   * \param controller_nh Node handle inside the controller namespace
   */
  void setOdomPubFields(ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh);
};

PLUGINLIB_EXPORT_CLASS(tricycle_controller::TricycleController,
                       controller_interface::ControllerBase);

} // namespace tricycle_controller
