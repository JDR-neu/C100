/*
 *  Created on: Jan 1, 2018
 *      Author: shansu
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef __IR100HW_H_
#define __IR100HW_H_

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ir100_driver/ir100_diagnostics.h>
#include <ir100_driver/motor_controller.h>
#include <ir100_driver/bms.h>

// C100
#define SP 0.10471975512
#define PP 0.00153398078789
#define CP 31.8309886184
//#define SP 0.00349065850399
//#define PP 0.000261799387799
//#define CP 53.0516476973

namespace ir100
{

  class IR100HW : public hardware_interface::RobotHW
  {
    public:
      IR100HW(std::string controller_port, std::string bms_port, double diagnostic_period);
      void read();
      void write();

    public:
      MotorController *motor_controller;
      BMS *bms;

    protected:

    private:
      ros::NodeHandle nh;
      ros::ServiceClient set_robot_state_client;

      hardware_interface::JointStateInterface    js_interface_;
      hardware_interface::VelocityJointInterface vj_interface_;

      std::vector<double> joint_velocity_command_;
  
      std::vector<std::string> joint_name_;
      std::vector<double> joint_position_;
      std::vector<double> joint_velocity_;
      std::vector<double> joint_effort_;

    private:
      ir100::RobotStatusTask robot_status_task;
      ir100::BatteryStatusTask battery_status_task;
      ir100::MotorStatusTask motor_status_task;
      ir100::ControllerStatusTask controller_status_task;
      diagnostic_updater::Updater updater;

      int left_encoder_speed;
      int right_encoder_speed;

      int64_t left_encoder_counts;
      int64_t right_encoder_counts;

      double diagnostic_dt;
      ros::Time diagnostic_current_time, diagnostic_last_time, bms_last_time;
      bool emergency_stop;
      bool robot_state_set;
      bool robot_state_reset;
      int battery_capacity;
  };

}

#endif
