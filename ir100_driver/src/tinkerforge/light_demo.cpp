/*
 *  Created on: Jan 1, 2018
 *      Author: shansu
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <stdio.h>
#include <unistd.h>
#include <tinkerforge/ip_connection.h>
#include <tinkerforge/bricklet_led_strip_v2.h>
#include <ros/ros.h>


int main(int argc,char* argv[])
{
  ros::init(argc,argv,"light_demo");
  ros::NodeHandle nh;
  ros::NodeHandle *private_nh;
  private_nh = new ros::NodeHandle("~");
  std::string host,uid;
  int port;
  private_nh->param("host",host,std::string("192.168.99.30"));
  private_nh->param("port",port,4223);
  private_nh->param("uid",uid,std::string("FAT"));

  IPConnection ipcon;
  LEDStripV2 ls;
  // Create IP connection
  ipcon_create(&ipcon);

  // Create device object
  led_strip_v2_create(&ls, uid.c_str(), &ipcon);

  // Connect to brickd
  if(ipcon_connect(&ipcon, host.c_str(), port) < 0) {
    fprintf(stderr, "Could not connect\n");
  }
  // Don't use device before ipcon is connected

  led_strip_v2_set_chip_type(&ls,2812);

  uint8_t color[120] = {0};//40*3
  memset(color,0,120);

  led_strip_v2_set_led_values(&ls, 0, color, 120);

  if(ros::ok()) {
    std::cout<<"set color"<<std::endl;
    for(int i=0;i<120;i++) {
      color[i]    = 0;
      color[i++]  = 255;
      color[i++]  = 0;
    }
    led_strip_v2_set_led_values(&ls, 0, color, 120);
  }
  ros::spin();
  led_strip_v2_destroy(&ls);
  ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally
  return 0;
}
