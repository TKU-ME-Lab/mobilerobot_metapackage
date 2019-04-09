#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <map>
#include <string>

class CBT3000_Teleop_Joy
{
private:
  

  ros::Subscriber m_sub_joy;
  ros::Publisher  m_pub_cmd_vel;

  std::map<std::string, int> m_axis_linear_map;
  std::map<std::string, int> m_axis_angular_map;

  std::map<std::string, std::map<std::string, double> > m_scale_linear_map;
  std::map<std::string, std::map<std::string, double> > m_scale_angular_map;

  int m_enable_button;

  bool m_sent_disable_msg;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::string &which_map);

public:
  CBT3000_Teleop_Joy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);
};