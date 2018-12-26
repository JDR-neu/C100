/*
 *  Created on: Jan 1, 2018
 *      Author: shansu
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "sonar.h"
#include "std_msgs/UInt16.h"

using namespace ir100_driver;
int main(int argc,char* argv[])
{
  ros::init(argc,argv,"test");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  std::string left_uid,right_uid;
  private_nh.param("left_uid",left_uid,std::string("Efi"));
  private_nh.param("right_uid",right_uid,std::string("DYZ"));

  Sonar left("192.168.99.30","4223",left_uid);
  Sonar right("192.168.99.30","4223",right_uid);

  ros::Publisher left_pub = nh.advertise<std_msgs::UInt16>("/sonar/left",100);
  ros::Publisher right_pub = nh.advertise<std_msgs::UInt16>("/sonar/right",100);
  left.connect();
  right.connect();
  while(nh.ok()) {
    uint16_t distance;
    std_msgs::UInt16 msg_left,msg_right;
    left.getDistance(distance);
    msg_left.data = distance;
    left_pub.publish(msg_left);
    ros::Duration(0.1).sleep();
    //std::cout<<distance<<std::endl;
    right.getDistance(distance);
    //std::cout<<distance<<std::endl;
    msg_right.data = distance;
    right_pub.publish(msg_right);
    ros::Duration(0.1).sleep();
  }
  return 0;
}
