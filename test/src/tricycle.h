// NOTE: The contents of this file have been taken largely from the ros_control
// wiki tutorials

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>

class Tricycle : public hardware_interface::RobotHW {
public:
  Tricycle()
      : running_(true), start_srv_(nh_.advertiseService(
                            "start", &Tricycle::start_callback, this)),
        stop_srv_(
            nh_.advertiseService("stop", &Tricycle::stop_callback, this)) {
    std::string wheel_joint_name = "drive_wheel_joint";

    hardware_interface::JointStateHandle state_handle;
    // Connect and register the joint state and velocity interface
    state_handle = hardware_interface::JointStateHandle(
                                                        wheel_joint_name,
                                                        &wheel_joint_.position,
                                                        &wheel_joint_.velocity,
                                                        &wheel_joint_.effort );

    jnt_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(state_handle, &wheel_joint_.velocity_command);
    jnt_vel_interface_.registerHandle(vel_handle);

    std::string steer_joint_name = "steer_joint";
    // Connect and register the joint state and position interface
    state_handle = hardware_interface::JointStateHandle(
                                                        steer_joint_name,
                                                        &steer_joint_.position,
                                                        &steer_joint_.velocity,
                                                        &steer_joint_.effort );

    jnt_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(state_handle,
                                               &steer_joint_.position_command);
    jnt_pos_interface_.registerHandle(pos_handle);

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_pos_interface_);
  }

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read() {
    if (running_) {
      // Note that wheel_joint_.position will be NaN for one more cycle after we
      // start(), but that is consistent with the knowledge we have about the
      // state of the robot.
      wheel_joint_.position += wheel_joint_.velocity * getPeriod().toSec(); // update position
      wheel_joint_.velocity = wheel_joint_.velocity_command; // might add smoothing here later
      steer_joint_.position = steer_joint_.position_command; // might add smoothing here later
    } else {
      wheel_joint_.position = std::numeric_limits<double>::quiet_NaN();
      wheel_joint_.velocity = std::numeric_limits<double>::quiet_NaN();
      steer_joint_.position = std::numeric_limits<double>::quiet_NaN();
      steer_joint_.velocity = std::numeric_limits<double>::quiet_NaN();
    }
  }

  void write() {
    std::ostringstream os;

    os << wheel_joint_.velocity_command;

    ROS_DEBUG_STREAM("Commands for wheel joint: " << os.str());

    os.str("");

    os << steer_joint_.position_command;
    ROS_DEBUG_STREAM("Commands for steering joint: " << os.str());
  }

  bool start_callback(std_srvs::Empty::Request & /*req*/,
                      std_srvs::Empty::Response & /*res*/) {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request & /*req*/,
                     std_srvs::Empty::Response & /*res*/) {
    running_ = false;
    return true;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  struct Joint {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
  } wheel_joint_;

  struct SteeringJoint {
    double position;
    double velocity;
    double effort;
    double position_command;

    SteeringJoint()
        : position(0), velocity(0), effort(0), position_command(0) {}
  } steer_joint_;

  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
