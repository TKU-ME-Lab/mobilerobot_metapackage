#include <mobilerobot_control/codometry.h>
#include <tf/transform_datatypes.h>
#include <boost/bind.hpp>

namespace mecanum_wheel_controller
{
  COdometry::COdometry(size_t velocity_rolling_window_size):
    m_time_stamp(0.0), m_pose_x(0.0), m_pose_y(0.0), m_pose_heading(0.0),
    m_linear_x(0.0), m_linear_y(0.0), m_angular_z(0.0), m_wheels_k(0.0), m_wheels_radius(0.0),
    m_velocity_rolling_window_size(velocity_rolling_window_size),
    m_linear_X_Acc(RollingWindow::window_size = velocity_rolling_window_size),
    m_linear_Y_Acc(RollingWindow::window_size = velocity_rolling_window_size),
    m_angular_Z_Acc(RollingWindow::window_size = velocity_rolling_window_size),
    m_fIntegrate(boost::bind(&COdometry::integrateExact, this, _1, _2, _3))
  {
  }

  void COdometry::init(const ros::Time& time)
  {
    m_linear_X_Acc = RollingMeanAcc(RollingWindow::window_size = m_velocity_rolling_window_size);
    m_linear_Y_Acc = RollingMeanAcc(RollingWindow::window_size = m_velocity_rolling_window_size);
    m_angular_Z_Acc = RollingMeanAcc(RollingWindow::window_size = m_velocity_rolling_window_size);

    m_time_stamp = time;
  }

  bool COdometry::update(double wheel0_vel, double wheel1_vel, double wheel2_vel, double wheel3_vel, const ros::Time& time)
  {
    const double dt = (time - m_time_stamp).toSec();
    if (dt < 0.0001)
      return false;

    m_time_stamp = time;
    ROS_INFO_STREAM("Update Time:" + std::to_string(time.toSec()));
    m_linear_x = m_wheels_radius * ( wheel0_vel + wheel1_vel + wheel2_vel + wheel3_vel) * 0.25;
    m_linear_y = m_wheels_radius * (-wheel0_vel + wheel1_vel + wheel2_vel - wheel3_vel) * 0.25;
    m_angular_z = m_wheels_radius / m_wheels_k * (-wheel0_vel + wheel1_vel - wheel2_vel + wheel3_vel) * 0.25;

    m_fIntegrate(m_linear_x* dt, m_linear_y*dt, m_angular_z*dt);

    return true;
  }

  void COdometry::updateOpenLoop(double linearX, double linearY, double angularZ, const ros::Time &time)
  {
    m_linear_x = linearX;
    m_linear_y = linearY;
    m_angular_z = angularZ;

    const double dt = (time - m_time_stamp).toSec();
    m_time_stamp = time;
    ROS_INFO_STREAM("Update Open Time:" + std::to_string(time.toSec()));
    m_fIntegrate(linearX *dt, linearY *dt, angularZ *dt);
  }

  void COdometry::setWheelsParams(double wheels_k, double wheels_radius)
  {
    m_wheels_k = wheels_k;
    m_wheels_radius = wheels_radius;
  }

  void COdometry::integrateExact(double linearX, double linearY,double angular)
  {
    m_pose_heading += angular;

    tf::Matrix3x3 R_m_odom = tf::Matrix3x3(tf::createQuaternionFromYaw(m_pose_heading));
    tf::Vector3 vel_inOdom = R_m_odom * tf::Vector3(linearX, linearY, 0.0);

    m_pose_x += vel_inOdom.x();
    m_pose_y += vel_inOdom.y();

    ROS_INFO_STREAM("Pose X:" + std::to_string(m_pose_x) + ", Y:" + std::to_string(m_pose_y) + ", Angle:" + std::to_string(m_pose_heading));
  }

} // namespace mecanum_wheel_controller