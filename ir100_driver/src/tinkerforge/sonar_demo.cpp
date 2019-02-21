#include "sonar.h"
#include "std_msgs/UInt16.h"

using namespace ir100_driver;
int main(int argc,char* argv[])
{
  ros::init(argc,argv,"test");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  std::string left_uid,right_uid;
  private_nh.param("left_uid",left_uid,std::string("DZr"));
  private_nh.param("right_uid",right_uid,std::string("Eg8"));

  Sonar left("192.168.99.30","4223",left_uid);
  Sonar right("192.168.99.30","4223",right_uid);

  ros::Publisher left_pub = nh.advertise<std_msgs::UInt16>("/sonar/left",100);
  ros::Publisher right_pub = nh.advertise<std_msgs::UInt16>("/sonar/right",100);
  left.connect();
  right.connect();
  while(nh.ok()) {
    std_msgs::UInt16 msg_left,msg_right;
    left.getDistance(msg_left.data);
    left_pub.publish(msg_left);
    right.getDistance(msg_left.data);
    right_pub.publish(msg_right);
    ros::Duration(1.0).sleep();
  }
  return 0;
}
