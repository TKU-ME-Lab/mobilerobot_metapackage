#include <ros/ros.h>
#include <bt3000_teleop_joy/cbt3000_teleop_joy.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_twitst_joy_node");

  ros::NodeHandle nh(""), nh_param("~");
  CBT3000_Teleop_Joy joy_teleop(&nh, &nh_param);

  while(ros::ok())
  {
    ros::spin();
  }

  return 0;
}