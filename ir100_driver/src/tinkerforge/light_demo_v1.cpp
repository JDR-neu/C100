/*
 *  Created on: Jan 1, 2018
 *      Author: shansu
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <stdio.h>
#include <unistd.h>
#include <tinkerforge/ip_connection.h>
#include <tinkerforge/bricklet_led_strip.h>
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
  private_nh->param("uid",uid,std::string("xff"));

  IPConnection ipcon;
  LEDStrip ls;
  // Create IP connection
  ipcon_create(&ipcon);

  // Create device object
  led_strip_create(&ls, uid.c_str(), &ipcon);

  // Connect to brickd
  if(ipcon_connect(&ipcon, host.c_str(), port) < 0) {
    fprintf(stderr, "Could not connect\n");
  }
  // Don't use device before ipcon is connected

  led_strip_set_chip_type(&ls,2812);

  uint8_t red[16] = {0};
  uint8_t green[16] = {0};
  uint8_t blue[16] = {0};// = {0};

  memset(red,0,16);//blue
  memset(green,0,16);//red
  memset(blue,0,16);//green
  led_strip_set_rgb_values(&ls, 0, 16, red, green, blue);//g,r,b
  led_strip_set_rgb_values(&ls, 16, 16, red, green, blue);
  led_strip_set_rgb_values(&ls, 32, 16, red, green, blue);

  if(ros::ok()) {
    memset(red,255,16);//blue
    memset(green,0,16);//red
    memset(blue,0,16);//green
    led_strip_set_rgb_values(&ls, 0, 16, red, green, blue);//g,r,b
    led_strip_set_rgb_values(&ls, 16, 16, red, green, blue);
    led_strip_set_rgb_values(&ls, 32, 16, red, green, blue);
  }
  ros::spin();
  led_strip_destroy(&ls);
  ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally
  return 0;
}
