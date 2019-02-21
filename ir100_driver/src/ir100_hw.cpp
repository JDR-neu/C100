#include <ir100_driver/ir100_hw.h>

namespace ir100
{

  IR100HW::IR100HW(std::string controller_port, std::string bms_port, double diagnostic_period)
  {
    using namespace hardware_interface;

    motor_controller = new MotorController(controller_port);
    motor_controller->open();

    bms = new BMS(bms_port);
    bms->connect();

    left_encoder_speed = 0;
    right_encoder_speed = 0;

    left_encoder_counts = 0;
    right_encoder_counts = 0;

    diagnostic_dt = diagnostic_period;
    diagnostic_current_time = ros::Time::now();
    diagnostic_last_time = ros::Time::now();

    emergency_stop = false;
    robot_state_set = false;
    robot_state_reset = false;
    battery_capacity = 0;

    joint_name_.resize(2);
    joint_position_.resize(2);
    joint_velocity_.resize(2);
    joint_effort_.resize(2);
    joint_velocity_command_.resize(2);

    joint_name_[0] = "left_wheel_joint";
    joint_name_[1] = "right_wheel_joint";

    for (unsigned int i = 0; i < joint_name_.size(); i++)
    {
      js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));    
      vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));
    }

    registerInterface(&js_interface_);
    registerInterface(&vj_interface_);

    updater.setHardwareID("IR100");
    updater.add(robot_status_task);
    updater.add(battery_status_task);
    updater.add(motor_status_task);
    updater.add(controller_status_task);
  }

  void IR100HW::read()
  {
    left_encoder_speed = motor_controller->getEncoderSpeed(1);
    right_encoder_speed = motor_controller->getEncoderSpeed(2);
    if (left_encoder_speed != -1 && right_encoder_speed != -1)
    {
      // C100
      //joint_velocity_[0] = left_encoder_speed*SP;
      //joint_velocity_[1] = -right_encoder_speed*SP;
      // R300
      joint_velocity_[0] = left_encoder_speed*SP;
      joint_velocity_[1] = -right_encoder_speed*SP;
    }

    left_encoder_counts = motor_controller->getEncoderCount(1);
    right_encoder_counts = motor_controller->getEncoderCount(2);
    if (left_encoder_counts != -1 && right_encoder_counts != -1)
    {
      // C100
      //joint_position_[0] = left_encoder_counts*PP;
      //joint_position_[1] = -right_encoder_counts*PP;
      // R300
      joint_position_[0] = left_encoder_counts*PP;
      joint_position_[1] = -right_encoder_counts*PP;
    }

    diagnostic_current_time = ros::Time::now();
    if ((diagnostic_current_time - diagnostic_last_time).toSec() > diagnostic_dt)
    {
      float left_motor_current = motor_controller->getMotorCurrent(1);
      float right_motor_current = motor_controller->getMotorCurrent(2);
      float battery_voltage = motor_controller->getBatteryVoltage();
      float controller_temp = motor_controller->getControllerTemp();

      ros::Duration bms_period(0.1);
      if ((diagnostic_current_time - bms_last_time).toSec() > bms_period.toSec())
      {
        float soc = bms->getSOC();
        if (soc > 0)
        {
          soc = (soc > 100) ? 100 : soc;
          soc = (soc < 0) ? 0 : soc;
          battery_capacity = soc;
        }
        bms_last_time = diagnostic_current_time;
      }

      robot_status_task.update(emergency_stop);
      battery_status_task.update(battery_voltage, battery_capacity);
      motor_status_task.update(left_motor_current, right_motor_current, left_encoder_counts, right_encoder_counts);
      controller_status_task.update(controller_temp);
      updater.force_update();

      diagnostic_last_time = diagnostic_current_time;
    }
  }

  void IR100HW::write()
  {
    if (emergency_stop)
    {
      motor_controller->setMotorSpeed(1, 0);
      motor_controller->setMotorSpeed(2, 0);
    }
    else
    {
      // C100
      motor_controller->setMotorSpeed(1, joint_velocity_command_[0]*CP);
      motor_controller->setMotorSpeed(2, -joint_velocity_command_[1]*CP);
    }
  }

}
