#include <ros/ros.h>
#include "tricycle_driver.cpp"
#include <controller_manager/controller_manager.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tricycle_driver");
  ros::NodeHandle nh;

  Tricycle robot;
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    robot.read();
    robot.write();

    cm.update(robot.getTime(), robot.getPeriod());

    rate.sleep();
  }

}
