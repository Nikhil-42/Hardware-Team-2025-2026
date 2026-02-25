#pragma once
#include <cmath>

class PID
{
	public:
		PID(double kp, double ki, double kd, double max_output) :
		kp_(kp), ki_(ki), kd_(kd), max_output_(max_output) {}

		float compute(double error, double dt)
		{

			integrator_ += error * dt;
			double derivative = (error - prev_error_)/dt;

			double output = kp_ * error + ki_ * integrator_ + kd_ * derivative;
			prev_error_ = error;

			// clamp output (may need to clamp integator later)
			if(std::abs(output) > max_output_)
			{
				output = (output < 0) ? -max_output_ : max_output_;
			}
			//printf("Integrator: %f\t Derivative: %f\t dt: %f\n", integrator_, derivative, dt);
			return output;
		}

		void reset()
		{
			integrator_ = 0.0;
			prev_error_ = 0.0;
		}

	private:
		double kp_;
		double ki_;
		double kd_;

		double max_output_;
		double integrator_;
		double prev_error_;
};
