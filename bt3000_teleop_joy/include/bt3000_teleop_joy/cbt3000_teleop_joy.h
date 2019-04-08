#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class CBT3000_Teleop_Joy
{
private:
  

  ros::Subscriber m_sub_joy;
  ros::Publisher  m_pub_cmd_vel;


  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::string &which_map);

public:
  CBT3000_Teleop_Joy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);
};