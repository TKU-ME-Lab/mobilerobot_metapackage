#include <tf/transform_datatypes.h>
#include <transmission_interface/transmission_parser.h>
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <mobilerobot_control/cmecanum_wheel_controller.h>
#include <iostream>

static bool isCylinderOrSphere(const urdf::LinkConstSharedPtr &link)
{
  if (!link)
  {
    ROS_ERROR("Link == NULL");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link:" << link->name << " does not have collision description. Add collision description for link to urdf");
    return false;
  }
  
  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }

  if(link->collision->geometry->type != urdf::Geometry::CYLINDER && link->collision->geometry->type != urdf::Geometry::SPHERE)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder nor sphere geometry");
    return false;
  }

  return true;
}

namespace mecanum_wheel_controller
{
  CMecanumWheelController::CMecanumWheelController():
    m_open_loop(false), m_struct_command(), m_use_realigned_rooler_joints(false),
    m_wheels_k(0.0), m_wheels_radius(0.0), m_cmd_vel_timeout(0.5), m_mechanical_reduction(1.0),
    m_base_frame_id("base_link"), m_enable_odom_if(true), m_wheel_joints_size(0)
  {
  }

  bool CMecanumWheelController::init(hardware_interface::VelocityActuatorInterface* hw,
                                    ros::NodeHandle& nh, ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    m_name = complete_ns.substr(id+1);

    controller_nh.param("use_realigned_roller_joints", m_use_realigned_rooler_joints, m_use_realigned_rooler_joints);
    ROS_INFO_STREAM_NAMED(m_name, "Use the roller's radius rather than the wheel's: " << m_use_realigned_rooler_joints << ".");

    std::string wheel0_name;
    controller_nh.param("front_left_wheel_joint", wheel0_name, wheel0_name);
    ROS_INFO_STREAM_NAMED(m_name, "Front left wheel joint (wheel0) is : " << wheel0_name);

    std::string wheel1_name;
    controller_nh.param("front_right_wheel_joint", wheel1_name, wheel1_name);
    ROS_INFO_STREAM_NAMED(m_name, "Back left wheel joint (wheel1) is : " << wheel1_name);

    std::string wheel2_name;
    controller_nh.param("back_left_wheel_joint", wheel2_name, wheel2_name);
    ROS_INFO_STREAM_NAMED(m_name, "Back right wheel joint (wheel2) is : " << wheel2_name);

    std::string wheel3_name;
    controller_nh.param("back_right_wheel_joint", wheel3_name, wheel3_name);
    ROS_INFO_STREAM_NAMED(m_name, "Front right wheel joint (wheel3) is : " << wheel3_name);

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(m_name, "Controller state will be published at "
                          << publish_rate << "Hz.");
    m_publish_period = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", m_open_loop, m_open_loop);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", m_cmd_vel_timeout, m_cmd_vel_timeout);
    ROS_INFO_STREAM_NAMED(m_name, "Velocity commands will be considered old if they are older than "
                          << m_cmd_vel_timeout << "s.");

    controller_nh.param("base_frame_id", m_base_frame_id, m_base_frame_id);
    ROS_INFO_STREAM_NAMED(m_name, "Base frame_id set to " << m_base_frame_id);

    controller_nh.param("enable_odom_tf", m_enable_odom_if, m_enable_odom_if);
    ROS_INFO_STREAM_NAMED(m_name, "Publishing to tf is " << (m_enable_odom_if?"enabled":"disabled"));

    controller_nh.param("linear/x/has_velocity_limits"    , m_limiter_linear_X.m_has_velocity_limits    , m_limiter_linear_X.m_has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", m_limiter_linear_X.m_has_accelation_limits  , m_limiter_linear_X.m_has_accelation_limits);
    controller_nh.param("linear/x/max_velocity"           , m_limiter_linear_X.m_max_velocity           ,  m_limiter_linear_X.m_max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , m_limiter_linear_X.m_min_velocity           , -m_limiter_linear_X.m_max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , m_limiter_linear_X.m_max_acceleration       ,  m_limiter_linear_X.m_max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , m_limiter_linear_X.m_min_acceleration       , -m_limiter_linear_X.m_max_acceleration      );

    controller_nh.param("linear/y/has_velocity_limits"    , m_limiter_linear_Y.m_has_velocity_limits    , m_limiter_linear_Y.m_has_velocity_limits    );
    controller_nh.param("linear/y/has_acceleration_limits", m_limiter_linear_Y.m_has_accelation_limits  , m_limiter_linear_Y.m_has_accelation_limits  );
    controller_nh.param("linear/y/max_velocity"           , m_limiter_linear_Y.m_max_velocity           ,  m_limiter_linear_Y.m_max_velocity          );
    controller_nh.param("linear/y/min_velocity"           , m_limiter_linear_Y.m_min_velocity           , -m_limiter_linear_Y.m_max_velocity          );
    controller_nh.param("linear/y/max_acceleration"       , m_limiter_linear_Y.m_max_acceleration       ,  m_limiter_linear_Y.m_max_acceleration      );
    controller_nh.param("linear/y/min_acceleration"       , m_limiter_linear_Y.m_min_acceleration       , -m_limiter_linear_Y.m_max_acceleration      );

    controller_nh.param("angular/z/has_velocity_limits"    , m_limiter_angular_Z.m_has_velocity_limits    , m_limiter_angular_Z.m_has_velocity_limits);
    controller_nh.param("angular/z/has_acceleration_limits", m_limiter_angular_Z.m_has_accelation_limits  , m_limiter_angular_Z.m_has_accelation_limits);
    controller_nh.param("angular/z/max_velocity"           , m_limiter_angular_Z.m_max_velocity           ,  m_limiter_angular_Z.m_max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , m_limiter_angular_Z.m_min_velocity           , -m_limiter_angular_Z.m_max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , m_limiter_angular_Z.m_max_acceleration       ,  m_limiter_angular_Z.m_max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , m_limiter_angular_Z.m_min_acceleration       , -m_limiter_angular_Z.m_max_acceleration      );

    controller_nh.param("mechanical_reduction", m_mechanical_reduction, m_mechanical_reduction);
    ROS_INFO_STREAM_NAMED(m_name, "Mechanical Reduction: " << m_mechanical_reduction);

    // Get the joint objects to use in the realtime loop
    m_actuatorHandle_wheel0 = hw->getHandle(wheel0_name);  // throws on failure
    m_actuatorHandle_wheel1 = hw->getHandle(wheel1_name);  // throws on failure
    m_actuatorHandle_wheel2 = hw->getHandle(wheel2_name);  // throws on failure
    m_actuatorHandle_wheel3 = hw->getHandle(wheel3_name);  // throws on failure

    if (!setWheelParamsFromUrdf(nh, controller_nh, wheel0_name, wheel1_name, wheel2_name, wheel3_name))
      return false;

    setupRtPublishersMsg(nh, controller_nh);

    m_sub_command = controller_nh.subscribe("cmd_vel", 1, &CMecanumWheelController::cmdVelCallback, this);

    return true;
  }

  void CMecanumWheelController::update(const ros::Time &time, const ros::Duration &period)
  {
    if (m_open_loop)
    {
      m_odometry.updateOpenLoop(m_last_cmd.linear_x, m_last_cmd.linear_y, m_last_cmd.angular_z, time);
    }
    else
    {
      // get velocity , and convert unit [rpm] to [rad/s] 
      double wheel0_vel =  m_actuatorHandle_wheel0.getVelocity()/ m_mechanical_reduction * M_PI / 30;
      double wheel1_vel = -m_actuatorHandle_wheel1.getVelocity()/ m_mechanical_reduction * M_PI / 30;
      double wheel2_vel =  m_actuatorHandle_wheel2.getVelocity()/ m_mechanical_reduction * M_PI / 30;
      double wheel3_vel = -m_actuatorHandle_wheel3.getVelocity()/ m_mechanical_reduction * M_PI / 30;

      if (std::isnan(wheel0_vel) || std::isnan(wheel1_vel) || std::isnan(wheel2_vel) || std::isnan(wheel3_vel))
        return;

      m_odometry.update(wheel0_vel, wheel1_vel, wheel2_vel, wheel3_vel, time);
    }

    const geometry_msgs::Quaternion orientation_(tf::createQuaternionMsgFromYaw(m_odometry.getHeading()));

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = m_base_frame_id;
    odom_msg.pose.pose.position.x = m_odometry.getPoseX();
    odom_msg.pose.pose.position.y = m_odometry.getPoseY();
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation = orientation_;
    odom_msg.twist.twist.linear.x = m_odometry.getlinearX();
    odom_msg.twist.twist.linear.y = m_odometry.getlinearY();
    odom_msg.twist.twist.angular.z = m_odometry.getAngularZ();
    m_pub_odometry.publish(odom_msg);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = m_base_frame_id;
    odom_trans.transform.translation.x = m_odometry.getPoseX();
    odom_trans.transform.translation.y = m_odometry.getPoseY();
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation      = orientation_;
    m_pub_tf.sendTransform(odom_trans);

    Commands current_cmd = *(m_command.readFromRT());
    const double dt = (time - current_cmd.stamp).toSec();

    if (dt > m_cmd_vel_timeout)
    {
      current_cmd.linear_x = 0.0;
      current_cmd.linear_y = 0.0;
      current_cmd.angular_z = 0.0;
    }

    const double cmd_dt(period.toSec());
    m_limiter_linear_X.limit(current_cmd.linear_x, m_last_cmd.linear_x, cmd_dt);
    m_limiter_linear_Y.limit(current_cmd.linear_y, m_last_cmd.linear_y, cmd_dt);
    m_limiter_angular_Z.limit(current_cmd.angular_z, m_last_cmd.angular_z, cmd_dt);
    m_last_cmd = current_cmd;

    // Caculate 4 wheel velocity , and  convert unit [m/s] / [rpm]
    const double w0_vel =  1.0 / m_wheels_radius * (current_cmd.linear_x - current_cmd.linear_y - m_wheels_k * current_cmd.angular_z) * m_mechanical_reduction * 30 / M_PI;
    const double w1_vel = -1.0 / m_wheels_radius * (current_cmd.linear_x + current_cmd.linear_y + m_wheels_k * current_cmd.angular_z) * m_mechanical_reduction * 30 / M_PI;
    const double w2_vel =  1.0 / m_wheels_radius * (current_cmd.linear_x + current_cmd.linear_y - m_wheels_k * current_cmd.angular_z) * m_mechanical_reduction * 30 / M_PI;
    const double w3_vel = -1.0 / m_wheels_radius * (current_cmd.linear_x - current_cmd.linear_y + m_wheels_k * current_cmd.angular_z) * m_mechanical_reduction * 30 / M_PI;

    m_actuatorHandle_wheel0.setCommand(w0_vel);
    m_actuatorHandle_wheel1.setCommand(w1_vel);
    m_actuatorHandle_wheel2.setCommand(w2_vel);
    m_actuatorHandle_wheel3.setCommand(w3_vel);
  }

  void CMecanumWheelController::starting(const ros::Time &time)
  {
    brake();

    m_last_state_publish_time = time;
    m_odometry.init(time);
  }

  void CMecanumWheelController::stopping(const ros::Time &time)
  {
    brake();
  }

  void CMecanumWheelController::brake()
  {
    m_actuatorHandle_wheel0.setCommand(0.0);
    m_actuatorHandle_wheel1.setCommand(0.0);
    m_actuatorHandle_wheel2.setCommand(0.0);
    m_actuatorHandle_wheel3.setCommand(0.0);
  }

  void CMecanumWheelController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      m_struct_command.angular_z = command.angular.z;
      m_struct_command.linear_x = command.linear.x;
      m_struct_command.linear_y = command.linear.y;
      m_struct_command.stamp = ros::Time::now();
      m_command.writeFromNonRT(m_struct_command);
      ROS_DEBUG_STREAM_NAMED(m_name,
                             "Added values to command. "
                              << "Ang: "   << m_struct_command.angular_z << ", "
                              << "Lin: "   << m_struct_command.linear_x << ", "
                              << "Lin: "   << m_struct_command.linear_y << ", "
                              << "Stamp: " << m_struct_command.stamp); 
    }
    else
    {
      ROS_ERROR_NAMED(m_name, "Can't accept new commands. Controller is not running.");
    }
  }

  bool CMecanumWheelController::setWheelParamsFromUrdf(ros::NodeHandle &nh, ros::NodeHandle &controller_nh,
                                                        const std::string& wheel0_name, const std::string& wheel1_name, 
                                                        const std::string& wheel2_name, const std::string& wheel3_name)
  {
    bool has_wheel_separation_x = controller_nh.getParam("wheel_separation_x", m_wheel_separation_x);
    bool has_wheel_seperation_y = controller_nh.getParam("wheel_separation_y", m_wheel_separation_y);

    if (has_wheel_separation_x != has_wheel_seperation_y)
    {
      ROS_ERROR_STREAM_NAMED(m_name, "Only one whel separation overrided");
      return false;
    }

    bool lookup_wheel_separation = !(has_wheel_separation_x && has_wheel_seperation_y);
    bool lookup_wheel_radius     = !controller_nh.getParam("wheel_radius", m_wheels_radius);

    if (lookup_wheel_separation || lookup_wheel_radius)
    {
      // Parse robot description
      const std::string model_param_name = "robot_description";
      bool res = nh.hasParam(model_param_name);
      std::string robot_model_str="";
      if(!res || !nh.getParam(model_param_name,robot_model_str))
      {
        ROS_ERROR_NAMED(m_name, "Robot descripion couldn't be retrieved from param server.");
        return false;
      }

      urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));
      // Get wheels position and compute parameter k_ (used in mecanum wheels IK).
      urdf::JointConstSharedPtr wheel0_urdfJoint(model->getJoint("forward_left_joint"));
      if(!wheel0_urdfJoint)
      {
        ROS_ERROR_STREAM_NAMED(m_name, wheel0_name
                              << " couldn't be retrieved from model description");
        return false;
      }
      urdf::JointConstSharedPtr wheel1_urdfJoint(model->getJoint("forward_right_joint"));
      if(!wheel1_urdfJoint)
      {
        ROS_ERROR_STREAM_NAMED(m_name, wheel1_name
                              << " couldn't be retrieved from model description");
        return false;
      }
      urdf::JointConstSharedPtr wheel2_urdfJoint(model->getJoint("backward_left_joint"));
      if(!wheel2_urdfJoint)
      {
        ROS_ERROR_STREAM_NAMED(m_name, wheel2_name
                              << " couldn't be retrieved from model description");
        return false;
      }
      urdf::JointConstSharedPtr wheel3_urdfJoint(model->getJoint("backward_right_joint"));
      if(!wheel3_urdfJoint)
      {
        ROS_ERROR_STREAM_NAMED(m_name, wheel3_name
                              << " couldn't be retrieved from model description");
        return false;
      }

      if (lookup_wheel_separation)
      {
        ROS_INFO_STREAM("wheel0 to origin: "  << wheel0_urdfJoint->parent_to_joint_origin_transform.position.x << ","
                                              << wheel0_urdfJoint->parent_to_joint_origin_transform.position.y << ", "
                                              << wheel0_urdfJoint->parent_to_joint_origin_transform.position.z);
        ROS_INFO_STREAM("wheel1 to origin: "  << wheel1_urdfJoint->parent_to_joint_origin_transform.position.x << ","
                                              << wheel1_urdfJoint->parent_to_joint_origin_transform.position.y << ", "
                                              << wheel1_urdfJoint->parent_to_joint_origin_transform.position.z);
        ROS_INFO_STREAM("wheel2 to origin: "  << wheel2_urdfJoint->parent_to_joint_origin_transform.position.x << ","
                                              << wheel2_urdfJoint->parent_to_joint_origin_transform.position.y << ", "
                                              << wheel2_urdfJoint->parent_to_joint_origin_transform.position.z);
        ROS_INFO_STREAM("wheel3 to origin: "  << wheel3_urdfJoint->parent_to_joint_origin_transform.position.x << ","
                                              << wheel3_urdfJoint->parent_to_joint_origin_transform.position.y << ", "
                                              << wheel3_urdfJoint->parent_to_joint_origin_transform.position.z);

        double wheel0_x = wheel0_urdfJoint->parent_to_joint_origin_transform.position.x;
        double wheel0_y = wheel0_urdfJoint->parent_to_joint_origin_transform.position.y;
        double wheel1_x = wheel1_urdfJoint->parent_to_joint_origin_transform.position.x;
        double wheel1_y = wheel1_urdfJoint->parent_to_joint_origin_transform.position.y;
        double wheel2_x = wheel2_urdfJoint->parent_to_joint_origin_transform.position.x;
        double wheel2_y = wheel2_urdfJoint->parent_to_joint_origin_transform.position.y;
        double wheel3_x = wheel3_urdfJoint->parent_to_joint_origin_transform.position.x;
        double wheel3_y = wheel3_urdfJoint->parent_to_joint_origin_transform.position.y;

        m_wheels_k = (-(-wheel0_x - wheel0_y) - (wheel1_x - wheel1_y) + (-wheel2_x - wheel2_y) + (wheel3_x - wheel3_y))
                    / 4.0;
      }
      else
      {
        ROS_INFO_STREAM("Wheel seperation in X: " << m_wheel_separation_x);
        ROS_INFO_STREAM("Wheel seperation in Y: " << m_wheel_separation_y);

        // The seperation is the total distance between the wheels in X and Y.

        m_wheels_k = (m_wheel_separation_x + m_wheel_separation_y) / 2.0;
      }

      if (lookup_wheel_radius)
      {
        // Get wheels radius
        double wheel0_radius = 0.0;
        double wheel1_radius = 0.0;
        double wheel2_radius = 0.0;
        double wheel3_radius = 0.0;

        if (!getWheelRadius(model, model->getLink(wheel0_urdfJoint->child_link_name), wheel0_radius) ||
            !getWheelRadius(model, model->getLink(wheel1_urdfJoint->child_link_name), wheel1_radius) ||
            !getWheelRadius(model, model->getLink(wheel2_urdfJoint->child_link_name), wheel2_radius) ||
            !getWheelRadius(model, model->getLink(wheel3_urdfJoint->child_link_name), wheel3_radius))
        {
          ROS_ERROR_STREAM_NAMED(m_name, "Couldn't retrieve wheels' radius");
          return false;
        }

        if (abs(wheel0_radius - wheel1_radius) > 1e-3 ||
            abs(wheel0_radius - wheel2_radius) > 1e-3 ||
            abs(wheel0_radius - wheel3_radius) > 1e-3)
        {
          ROS_ERROR_STREAM_NAMED(m_name, "Wheels radius are not egual");
          return false;
        }

        m_wheels_radius = wheel0_radius;
      }
    }

    ROS_INFO_STREAM("Wheel radius: " << m_wheels_radius);

    // Set wheel params for the odometry computation
    m_odometry.setWheelsParams(m_wheels_k, m_wheels_radius);

    return true;
  }
  

  bool CMecanumWheelController::getWheelRadius(const urdf::ModelInterfaceSharedPtr model,
                                               const urdf::LinkConstSharedPtr &wheel_link, double &wheel_radius)
  {
    urdf::LinkConstSharedPtr radius_link = wheel_link;

    if (m_use_realigned_rooler_joints)
    {
      const urdf::JointConstSharedPtr &roller_joint = radius_link->child_joints[0];

      if (!roller_joint)
      {
         ROS_ERROR_STREAM_NAMED(m_name, "No roller joint could be retrieved for wheel : " << wheel_link->name <<
          ". Are you sure mecanum wheels are simulated using realigned rollers?");
        return false;
      }

      radius_link = model->getLink(roller_joint->child_link_name);
      if(!radius_link)
      {
        ROS_ERROR_STREAM_NAMED(m_name, "No roller link could be retrieved for wheel : " << wheel_link->name <<
          ". Are you sure mecanum wheels are simulated using realigned rollers?");
        return false;
      }
    }

    if (!isCylinderOrSphere(radius_link))
    {
      ROS_ERROR_STREAM("Wheel link " << radius_link->name << " is NOT modeled as a cylinder!");
      return false;
    }

    if (radius_link->collision->geometry->type == urdf::Geometry::CYLINDER)
      wheel_radius = (static_cast<urdf::Cylinder*>(radius_link->collision->geometry.get()))->radius;
    else
      wheel_radius = (static_cast<urdf::Sphere*>(radius_link->collision->geometry.get()))->radius;

    return true;
  }

  void CMecanumWheelController::setupRtPublishersMsg(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry msg.
    m_pub_odometry = controller_nh.advertise<nav_msgs::Odometry>("odom", 50);
  }
} // mecanum_wheel_controller