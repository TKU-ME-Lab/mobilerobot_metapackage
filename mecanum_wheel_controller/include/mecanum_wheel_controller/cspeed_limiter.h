

namespace mecanum_wheel_controller
{
	class CSpeedLimiter
	{
	public:
		bool m_has_velocity_limits;
		bool m_has_accelation_limits;

		double m_min_velocity;
		double m_max_velocity;

		double m_min_acceleration;
		double m_max_acceleration;

		CSpeedLimiter(
			bool has_velocity_limits = false,
			bool has_acceleration_limits = false,
			double min_velocity = 0.0,
			double max_velocity = 0.0,
			double min_acceleration = 0.0,
			double max_acceleration = 0.0
		);
        
		void limit(double &v, double v0, double dt);

		void limit_velocity(double &v);

		void limit_acceleration(double &v, double v0, double dt);
	};
} // mecanum_wheel_controller