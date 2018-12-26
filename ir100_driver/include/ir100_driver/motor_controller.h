/*
 *  Created on: Jan 1, 2018
 *      Author: shansu
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef __MOTORCONTROLLER_H_
#define __MOTORCONTROLLER_H_

#include <string>

#include <ir100_driver/RoboteqDevice.h>
#include <ir100_driver/Constants.h>
#include <ir100_driver/ErrorCodes.h>

#define DIGITAL_INPUTS  4

class MotorController {

  private:
    RoboteqDevice *controller;
    std::string port;

  public:
    MotorController(std::string dev);
    ~MotorController();

    int open();
    int close();

    float getBatteryVoltage();
    float getControllerTemp();
    int getDigitalInput(int number);
    int getEncoderCount(int channel);
    int getEncoderSpeed(int channel);
    float getMotorCurrent(int channel);
    int setMotorSpeed(int channel, int speed);
};

#endif
