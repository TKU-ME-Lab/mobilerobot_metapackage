#include <bt3000_teleop_joy/cbt3000_teleop_joy.h>

CBT3000_Teleop_Joy::CBT3000_Teleop_Joy(ros::NodeHandle* nh, ros::NodeHandle *nh_param):m_sent_disable_msg(false)
{

  m_pub_cmd_vel = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  m_sub_joy     = nh->subscribe<sensor_msgs::Joy>("joy", 1, &CBT3000_Teleop_Joy::joyCallback, this);

  nh_param->param<int>("enable_button", m_enable_button, 0);

  if (nh_param->getParam("axis_linear", m_axis_linear_map))
  {
    nh_param->getParam("scale_linear", m_scale_linear_map["normal"]);
  }
  else
  {
    nh_param->param<int>("axis_linear", m_axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", m_scale_linear_map["normal"]["x"], 0.5);
  }

  if (nh_param->getParam("axis_angular", m_axis_angular_map))
  {
    nh_param->getParam("scale_angular", m_scale_angular_map["normal"]);
  }
  else
  {
    nh_param->param<int>("scale_angular", m_axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", m_scale_angular_map["normal"]["yaw"], 0.5);
  }

  for (std::map<std::string, int>::iterator it = m_axis_linear_map.begin(); it != m_axis_linear_map.end(); it++)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.", it->first.c_str(), it->second, m_scale_linear_map["normal"][it->first]);
  }
  
  for (std::map<std::string, int>::iterator it = m_axis_angular_map.begin(); it != m_axis_angular_map.end(); it++)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.", it->first.c_str(), it->second, m_scale_angular_map["normal"][it->first]);
  }
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }
  
  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void CBT3000_Teleop_Joy::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                       const std::string& which_map)
{
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x  = getVal(joy_msg, m_axis_linear_map, m_scale_linear_map[which_map], "x");
  cmd_vel_msg.linear.y  = getVal(joy_msg, m_axis_linear_map, m_scale_linear_map[which_map], "y");
  cmd_vel_msg.linear.z  = getVal(joy_msg, m_axis_linear_map, m_scale_linear_map[which_map], "z");
  cmd_vel_msg.angular.x = getVal(joy_msg, m_axis_angular_map, m_scale_angular_map[which_map], "roll");
  cmd_vel_msg.angular.y = getVal(joy_msg, m_axis_angular_map, m_scale_angular_map[which_map], "pitch");
  cmd_vel_msg.angular.z = getVal(joy_msg, m_axis_angular_map, m_scale_angular_map[which_map], "yaw");

  m_pub_cmd_vel.publish(cmd_vel_msg);
  m_sent_disable_msg = false;
}

void CBT3000_Teleop_Joy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons.size() > m_enable_button &&
      joy_msg->buttons[m_enable_button])
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    if (!m_sent_disable_msg)
    {
      geometry_msgs::Twist cmd_vel_msg;
      m_pub_cmd_vel.publish(cmd_vel_msg);
      m_sent_disable_msg = true;
    }
  }
  
}