#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <urdf_parser/urdf_parser.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <mobilerobot_control/codometry.h>
#include <mobilerobot_control/cspeed_limiter.h>

namespace mecanum_wheel_controller
{
  class CMecanumWheelController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  private:
    std::string m_name;

    ros::Duration m_publish_period;
    ros::Time     m_last_state_publish_time;
    
    bool m_open_loop;

    hardware_interface::JointHandle m_jointHandle_wheel0;
    hardware_interface::JointHandle m_jointHandle_wheel1;
    hardware_interface::JointHandle m_jointHandle_wheel2;
    hardware_interface::JointHandle m_jointHandle_wheel3;

    struct  Commands
    {
      double linear_x;
      double linear_y;
      double angular_z;
      ros::Time stamp;

      Commands(): linear_x(0.0), linear_y(0.0), angular_z(0.0), stamp(0.0) {}
    };

    realtime_tools::RealtimeBuffer<Commands> m_command;
    Commands m_struct_command;

    ros::Subscriber m_sub_command;

    ros::Publisher     m_pub_odometry;
    tf::TransformBroadcaster m_tf_broadcaster;

    COdometry m_odometry;
    geometry_msgs::TransformStamped m_odom_frame;

    bool m_use_realigned_rooler_joints;
    double m_wheels_k;
    double m_wheels_radius;
    double m_wheel_separation_x;
    double m_wheel_separation_y;

    double m_cmd_vel_timeout;

    std::string m_base_frame_id;

    bool m_enable_odom_if;

    size_t m_wheel_joints_size;

    Commands m_last_cmd;
    CSpeedLimiter m_limiter_linear_X;
    CSpeedLimiter m_limiter_linear_Y;
    CSpeedLimiter m_limiter_angular_Z;
    
    void brake();

    void cmdVelCallback(const geometry_msgs::Twist& );
    bool setWheelParamsFromUrdf(ros::NodeHandle&, ros::NodeHandle&,
                                const std::string&,
                                const std::string&,
                                const std::string&,
                                const std::string&);

    bool getWheelRadius(const urdf::ModelInterfaceSharedPtr model, const urdf::LinkConstSharedPtr& wheel_link, double& wheel_radius);

    void setupRtPublishersMsg(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);


  public:
    CMecanumWheelController();

    bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);
  };

  PLUGINLIB_EXPORT_CLASS(mecanum_wheel_controller::CMecanumWheelController, controller_interface::ControllerBase)
}
