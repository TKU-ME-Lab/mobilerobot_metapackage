#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Path path_msgs;

void odom_Callback(const nav_msgs::OdometryConstPtr &msg)
{
  path_msgs.header = msg->header;
  geometry_msgs::PoseStamped this_pose;
  this_pose.pose.position = msg->pose.pose.position;
  path_msgs.poses.push_back(this_pose);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "showpath");
  ros::NodeHandle nh("");

  ros::Subscriber sub_odom = nh.subscribe("/mobile_chassis_velocity_controller/odom", 1, &odom_Callback);
  ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("trajectory", 10);

  path_msgs.header.frame_id = "odom";
  path_msgs.header.stamp = ros::Time::now();

  size_t pose_num_last = path_msgs.poses.size();

  ros::Rate Rate(1);
  while(ros::ok())
  {
    size_t pose_num_now = path_msgs.poses.size();
    if (pose_num_now > pose_num_last)
    {
      pub_path.publish(path_msgs);
      pose_num_last = pose_num_now;
    }
    
    ros::spinOnce();
    Rate.sleep();
  }
  
  return 0;
}