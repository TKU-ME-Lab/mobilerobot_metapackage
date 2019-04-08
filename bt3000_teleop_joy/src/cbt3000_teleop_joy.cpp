#include <bt3000_teleop_joy/cbt3000_teleop_joy.h>

CBT3000_Teleop_Joy::CBT3000_Teleop_Joy(ros::NodeHandle* nh, ros::NodeHandle *nh_param)
{

  m_pub_cmd_vel = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  m_sub_joy     = nh->subscribe<sensor_msgs::Joy>("joy", 1, &CBT3000_Teleop_Joy::joyCallback, this);

  
}