// License: BSD
// https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE

#include <cmath>

#include <tf/transform_datatypes.h>

#include <boost/assign.hpp>

#include <tricycle_controller/tricycle_controller.h>
#include <urdf_geometry_parser/urdf_geometry_parser.h>


namespace tricycle_controller {

TricycleController::TricycleController()
    : command_struct_(), wheel_radius_(0.0), wheel_base_(0.0),
      cmd_vel_timeout_(0.5), base_frame_id_("base_footprint"),
      odom_frame_id_("odom"), enable_odom_tf_(true), angle_tolerance_(0.2) {}

bool TricycleController::init(hardware_interface::RobotHW *robot_hw,
                              ros::NodeHandle &root_nh,
                              ros::NodeHandle &controller_nh) {
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // Get joint name from the parameter server
  std::string drive_wheel_joint_name, steer_joint_name;

  if (!getJointName(controller_nh, "drive_wheel_joint",
                    drive_wheel_joint_name) ||
      !getJointName(controller_nh, "steer_joint", steer_joint_name)) {
    ROS_ERROR_STREAM("Get joint names from param failed");
    return false;
  }

  // Odometry related:
  double publish_rate;
  controller_nh.param("publish_rate", publish_rate, 50.0);
  ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                                   << publish_rate << "Hz.");
  publish_period_ = ros::Duration(1.0 / publish_rate);

  controller_nh.param("open_loop", open_loop_, open_loop_);

  int velocity_rolling_window_size = 10;
  controller_nh.param("velocity_rolling_window_size",
                      velocity_rolling_window_size,
                      velocity_rolling_window_size);
  ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                                   << velocity_rolling_window_size << ".");

  odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

  // Twist command related:
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
  ROS_INFO_STREAM_NAMED(
      name_, "Velocity commands will be considered old if they are older than "
                 << cmd_vel_timeout_ << "s.");

  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  ROS_INFO_STREAM_NAMED(name_,
                        "Publishing to tf is "
                            << (enable_odom_tf_ ? "enabled" : "disabled"));

  // Velocity and acceleration limits:
  controller_nh.param("linear/x/has_velocity_limits",
                      limiter_lin_.has_velocity_limits,
                      limiter_lin_.has_velocity_limits);
  controller_nh.param("linear/x/has_acceleration_limits",
                      limiter_lin_.has_acceleration_limits,
                      limiter_lin_.has_acceleration_limits);
  controller_nh.param("linear/x/max_velocity", limiter_lin_.max_velocity,
                      limiter_lin_.max_velocity);
  controller_nh.param("linear/x/min_velocity", limiter_lin_.min_velocity,
                      -limiter_lin_.max_velocity);
  controller_nh.param("linear/x/max_acceleration",
                      limiter_lin_.max_acceleration,
                      limiter_lin_.max_acceleration);
  controller_nh.param("linear/x/min_acceleration",
                      limiter_lin_.min_acceleration,
                      -limiter_lin_.max_acceleration);

  controller_nh.param("angular/z/has_velocity_limits",
                      limiter_ang_.has_velocity_limits,
                      limiter_ang_.has_velocity_limits);
  controller_nh.param("angular/z/has_acceleration_limits",
                      limiter_ang_.has_acceleration_limits,
                      limiter_ang_.has_acceleration_limits);
  controller_nh.param("angular/z/max_velocity", limiter_ang_.max_velocity,
                      limiter_ang_.max_velocity);
  controller_nh.param("angular/z/min_velocity", limiter_ang_.min_velocity,
                      -limiter_ang_.max_velocity);
  controller_nh.param("angular/z/max_acceleration",
                      limiter_ang_.max_acceleration,
                      limiter_ang_.max_acceleration);
  controller_nh.param("angular/z/min_acceleration",
                      limiter_ang_.min_acceleration,
                      -limiter_ang_.max_acceleration);

  // If either parameter is not available, we need to look up the value in the
  // URDF
  bool lookup_wheel_radius =
      !controller_nh.getParam("wheel_radius", wheel_radius_);
  bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);

  urdf_geometry_parser::UrdfGeometryParser uvk(root_nh, base_frame_id_);

  if (lookup_wheel_radius)
    if (!uvk.getJointRadius(drive_wheel_joint_name, wheel_radius_)) {
      ROS_ERROR_STREAM("Failed to get wheel radius, aborting.");
      return false;
    } else
      controller_nh.setParam("wheel_radius", wheel_radius_);

  if (lookup_wheel_base)
    // Does not take z into account
    // TODO: make more general, use getTransformVector maybe?
    if (!uvk.getDistanceBetweenJoints("base_footprint_joint", steer_joint_name,
                                      wheel_base_)) {
      ROS_ERROR_STREAM("Failed to get position of steer joint, aborting.");
      return false;
    } else
      controller_nh.setParam("wheel_base", wheel_base_);

  // Regardless of how we got the separation and radius, use them
  // to set the odometry parameters
  odometry_.setWheelParams(wheel_radius_, wheel_base_);
  ROS_INFO_STREAM_NAMED(name_, "Odometry params : "
                                   << ", wheel radius " << wheel_radius_
                                   << ", wheel base " << wheel_base_);

  drive_wheel_kinematics_.setMountPose(wheel_base_, 0);
  drive_wheel_kinematics_.setRadius(wheel_radius_);

  setOdomPubFields(root_nh, controller_nh);

  hardware_interface::VelocityJointInterface *const vel_joint_hw =
      robot_hw->get<hardware_interface::VelocityJointInterface>();

  hardware_interface::PositionJointInterface *const pos_joint_hw =
      robot_hw->get<hardware_interface::PositionJointInterface>();

  ROS_INFO_STREAM_NAMED(
      name_, "Adding drive wheel with joint name: " << drive_wheel_joint_name);
  drive_wheel_joint_ =
      vel_joint_hw->getHandle(drive_wheel_joint_name); // throws on failure

  ROS_INFO_STREAM_NAMED(
      name_, "Adding steering with joint name: " << steer_joint_name);

  steer_joint_ = pos_joint_hw->getHandle(steer_joint_name); // throws on failure

  sub_command_ = controller_nh.subscribe(
      "cmd_vel", 1, &TricycleController::cmdVelCallback, this);

  return true;
}

void TricycleController::update(const ros::Time &time,
                                const ros::Duration &period) {
  updateOdometry(time);
  updateCommand(time, period);
}

void TricycleController::starting(const ros::Time &time) {
  brake();

  // Register starting time used to keep fixed rate
  last_state_publish_time_ = time;

  odometry_.init(time);
}

void TricycleController::stopping(const ros::Time & /*time*/) { brake(); }

void TricycleController::updateOdometry(const ros::Time &time) {
  // Compute odometry
  if (open_loop_) {
    odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
  }

  const double steer_pos = steer_joint_.getPosition();
  const double drive_wheel_pos = drive_wheel_joint_.getPosition();

  odometry_.update(steer_pos, drive_wheel_pos, time);

  ROS_DEBUG("Odom: x: %f, y: %f, heading: %f", odometry_.getX(),
            odometry_.getY(), odometry_.getHeading());

  // Publish odometry message
  if (last_state_publish_time_ + publish_period_ < time) {
    last_state_publish_time_ += publish_period_;
    // Compute and store orientation info
    const geometry_msgs::Quaternion orientation(
        tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

    // Populate odom message and publish
    if (odom_pub_->trylock()) {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
      odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
      odom_pub_->msg_.pose.pose.orientation = orientation;
      odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinear();
      odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
      odom_pub_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (enable_odom_tf_ && tf_odom_pub_->trylock()) {
      geometry_msgs::TransformStamped &odom_frame =
          tf_odom_pub_->msg_.transforms[0];
      odom_frame.header.stamp = time;
      odom_frame.transform.translation.x = odometry_.getX();
      odom_frame.transform.translation.y = odometry_.getY();
      odom_frame.transform.rotation = orientation;
      tf_odom_pub_->unlockAndPublish();
    }
  }
}

void TricycleController::updateCommand(const ros::Time &time,
                                       const ros::Duration &period) {
  // Retreive current velocity command and time step:
  Command curr_cmd = *(command_.readFromRT());
  const double dt = (time - curr_cmd.stamp).toSec();

  // Brake if cmd_vel has timeout:
  if (dt > cmd_vel_timeout_) {
    curr_cmd.lin = 0.0;
    curr_cmd.ang = 0.0;
  }

  const double cmd_dt(period.toSec());

  const double angular_speed = odometry_.getAngular();

  // Limit velocities and accelerations:
  limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
  limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

  last1_cmd_ = last0_cmd_;
  last0_cmd_ = curr_cmd;

  drive_wheel_kinematics_.setCommand(curr_cmd.lin, curr_cmd.ang);
  drive_wheel_kinematics_.update();

  double steer_angle = drive_wheel_kinematics_.getSteer();
  double steer_angle_state = steer_joint_.getPosition();

  double drive_wheel_vel = drive_wheel_kinematics_.getAngularVelocity();

  if(fabs(steer_angle_state - steer_angle) > angle_tolerance_)
  {
    drive_wheel_vel = 0;
  }

  drive_wheel_joint_.setCommand(drive_wheel_vel);
  steer_joint_.setCommand(steer_angle);
}

void TricycleController::brake() {
  const double vel = 0.0;
  drive_wheel_joint_.setCommand(vel);

  // const double pos = 0.0;
  // steer_joint_.setCommand(pos);
}

void TricycleController::cmdVelCallback(const geometry_msgs::Twist &command) {
  if (isRunning()) {
    // check that we don't have multiple publishers on the command topic
    if (!allow_multiple_cmd_vel_publishers_ &&
        sub_command_.getNumPublishers() > 1) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(
          1.0, name_,
          "Detected "
              << sub_command_.getNumPublishers()
              << " publishers. Only 1 publisher is allowed. Going to brake.");
      brake();
      return;
    }

    command_struct_.ang = command.angular.z;
    command_struct_.lin = command.linear.x;
    command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT(command_struct_);
    ROS_DEBUG_STREAM_NAMED(name_, "Added values to command. "
                                      << "Ang: " << command_struct_.ang << ", "
                                      << "Lin: " << command_struct_.lin << ", "
                                      << "Stamp: " << command_struct_.stamp);
  } else {
    ROS_ERROR_NAMED(name_,
                    "Can't accept new commands. Controller is not running.");
  }
}

bool TricycleController::getJointName(ros::NodeHandle &controller_nh,
                                      const std::string &wheel_param,
                                      std::string &wheel_name) {
  XmlRpc::XmlRpcValue wheel_list;
  if (!controller_nh.getParam(wheel_param, wheel_list)) {
    ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve wheel param '"
                                      << wheel_param << "'.");
    return false;
  }

  if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    if (wheel_list.size() == 0) {
      ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param
                                                    << "' is an empty list");
      return false;
    }

    for (int i = 0; i < wheel_list.size(); ++i) {
      if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' #"
                                                      << i
                                                      << " isn't a string.");
        return false;
      }
    }
    wheel_name = static_cast<std::string>(wheel_list[0]);
  } else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
    wheel_name = static_cast<std::string>(wheel_list);
  } else {
    ROS_ERROR_STREAM_NAMED(
        name_, "Wheel param '"
                   << wheel_param
                   << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}

void TricycleController::setOdomPubFields(ros::NodeHandle &root_nh,
                                          ros::NodeHandle &controller_nh) {
  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  if (!controller_nh.getParam("pose_covariance_diagonal", pose_cov_list)) {
    ROS_ERROR_STREAM("Param pose_covariance_diagonal not found");
  }

  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  if (!controller_nh.getParam("twist_covariance_diagonal", twist_cov_list)) {
    ROS_ERROR_STREAM("Param twist_covariance_diagonal not found");
  }
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(
      controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = odom_frame_id_;
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;
  odom_pub_->msg_.pose.covariance =
      boost::assign::list_of(static_cast<double>(pose_cov_list[0]))(0)(0)(0)(0)(
          0)(0)(static_cast<double>(pose_cov_list[1]))(0)(0)(0)(0)(0)(0)(
          static_cast<double>(pose_cov_list[2]))(0)(0)(0)(0)(0)(0)(
          static_cast<double>(pose_cov_list[3]))(0)(0)(0)(0)(0)(0)(
          static_cast<double>(pose_cov_list[4]))(0)(0)(0)(0)(0)(0)(
          static_cast<double>(pose_cov_list[5]));
  odom_pub_->msg_.twist.twist.linear.y = 0;
  odom_pub_->msg_.twist.twist.linear.z = 0;
  odom_pub_->msg_.twist.twist.angular.x = 0;
  odom_pub_->msg_.twist.twist.angular.y = 0;
  odom_pub_->msg_.twist.covariance =
      boost::assign::list_of(static_cast<double>(twist_cov_list[0]))(0)(0)(0)(
          0)(0)(0)(static_cast<double>(twist_cov_list[1]))(0)(0)(0)(0)(0)(0)(
          static_cast<double>(twist_cov_list[2]))(0)(0)(0)(0)(0)(0)(
          static_cast<double>(twist_cov_list[3]))(0)(0)(0)(0)(0)(0)(
          static_cast<double>(twist_cov_list[4]))(0)(0)(0)(0)(0)(0)(
          static_cast<double>(twist_cov_list[5]));
  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(
      root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
}

} // namespace tricycle_controller
