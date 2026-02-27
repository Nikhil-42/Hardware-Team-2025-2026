#pragma once
#include <cmath>

class PID
{
	public:
		PID(double kp, double ki, double kd, double kf, double max_output) :
		kp_(kp), ki_(ki), kd_(kd), kf_(kf),  max_output_(max_output), integrator_(0.0), prev_error_(0.0) {}

		float compute(double setpoint, double error, double dt)
		{
			double unclamped_integrator = integrator_ + error * dt;
			double derivative = (error - prev_error_)/dt;

			double output = kf_ * setpoint + kp_ * error + ki_ * unclamped_integrator + kd_ * derivative;
			prev_error_ = error;

			// clamp output (may need to clamp integator later)
			if(std::abs(output) > max_output_)
			{
				output = (output < 0) ? -max_output_ : max_output_;
				// use integrator if it doesnt drive output further into saturation
				if(!((output > 0 && error > 0) || (output < 0 && error < 0)))
				{
					integrator_ = unclamped_integrator;
				}
			}
			else
			{
				//if output is saturated, don't want to windup the integral
					integrator_ = unclamped_integrator;
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
		double kf_;

		double max_output_;
		double integrator_;
		double prev_error_;
};
