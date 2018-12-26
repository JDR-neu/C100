/*
 *  Created on: Jan 1, 2018
 *      Author: shansu
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ir100_driver/ir100_hw.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ir100_driver");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh, private_nh("~");

  std::string controller_port;
  private_nh.param<std::string>("controller_port", controller_port, "/dev/ttyACM0");
  std::string bms_port;
  private_nh.param<std::string>("bms_port", bms_port, "/dev/ttyUSB0");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 50.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 4.0);
  ros::Duration control_period(1 / control_frequency);
  ros::Duration diagnostic_period(1 / diagnostic_frequency);

  ir100::IR100HW ir100(controller_port, bms_port, diagnostic_period.toSec());
  controller_manager::ControllerManager cm(&ir100, nh);

  while (ros::ok())
  {
    ir100.read();
    cm.update(ros::Time::now(), control_period);
    ir100.write();
  }
}

