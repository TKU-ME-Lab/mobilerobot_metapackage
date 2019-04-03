#include <ros/ros.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

using namespace boost::accumulators;

namespace mecanum_wheel_controller
{
  class COdometry
  {
    typedef accumulator_set<double, stats<tag::rolling_mean> > RollingMeanAcc;
    typedef tag::rolling_window RollingWindow;

    typedef boost::function<void(double, double, double)> IntegrationFunction;

  private:
    ros::Time m_time_stamp;

    double m_pose_x;
    double m_pose_y;
    double m_pose_heading;

    double m_linear_x;
    double m_linear_y;
    double m_angular_z;

    double m_wheels_k;
    double m_wheels_radius;

    size_t m_velocity_rolling_window_size;
    RollingMeanAcc m_linear_X_Acc;
    RollingMeanAcc m_linear_Y_Acc;
    RollingMeanAcc m_angular_Z_Acc;

    void integrateExact(double, double, double);

    IntegrationFunction m_fIntegrate;

  public:
    COdometry(size_t velocity_rolling_window_size = 10);

    void init(const ros::Time& time);

    bool update(double wheel0_vel, double wheel1_vel, double wheel2_vel, double wheel3_vel, const ros::Time& time);

    void updateOpenLoop(double linearX, double linearY, double angular, const ros::Time&time);

    double getHeading() const
    {
      return m_pose_heading;
    }

    double getPoseX() const
    {
      return m_pose_x;
    }
    
    double getPoseY() const
    {
      return m_pose_y;
    }

    double getlinearX() const
    {
      return m_linear_x;
    }

    double getlinearY() const 
    {
      return m_linear_y;
    }

    double getAngularZ() const 
    {
      return m_angular_z;
    }

    void setWheelsParams(double wheels_k, double wheels_radius);
  };
} //namespace mecanum_wheel_controller