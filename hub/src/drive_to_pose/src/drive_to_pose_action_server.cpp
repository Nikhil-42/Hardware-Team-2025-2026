#include <thread>
#include <cmath>

<<<<<<< HEAD
#include "dtp_interfaces/action/drive_to_pose.hpp"
=======
#include "hub_interfaces/action/drive_to_pose.hpp"
>>>>>>> f85e9d60a3f9b9e54e0df42ecb0a464a98d85ba5
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "drive_to_pose/pid.hpp"

class DriveToPoseServer : public rclcpp::Node
{
<<<<<<< HEAD
	using DriveToPose = dtp_interfaces::action::DriveToPose;
=======
	using DriveToPose = hub_interfaces::action::DriveToPose;
>>>>>>> f85e9d60a3f9b9e54e0df42ecb0a464a98d85ba5
	using GoalHandleDTP = rclcpp_action::ServerGoalHandle<DriveToPose>;

	public:
		// constructor for class that initializes the node as the action server
		explicit DriveToPoseServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
		  : Node("drive_to_pose_action_server", options),
		    PID_x_(1.0, 0.0, 0.0, 0.5),
		    PID_y_(1.0, 0.0, 0.0, 0.5),
		    PID_yaw_(1.0, 0.0, 0.0, 2.0)
		{
			cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

			auto odom_callback = [this](nav_msgs::msg::Odometry odom) -> void
			{
				current_x_ = odom.pose.pose.position.x;
				current_y_ = odom.pose.pose.position.y;
				const auto q = odom.pose.pose.orientation;

				//converting from quaternion to tait-bryan angles
				current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
			};
			odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10, odom_callback);

			auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid,
				std::shared_ptr<const DriveToPose::Goal> goal)
			{
				RCLCPP_INFO(this->get_logger(), "Recieved goal request with linear tolerance %f", goal->position_tolerance);
				(void)uuid;
				return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
			};

			auto handle_cancel = [this](const std::shared_ptr<GoalHandleDTP> goal_handle)
			{
				RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal");
				kill_robot();
				(void)goal_handle;
				return rclcpp_action::CancelResponse::ACCEPT;
			};

			auto handle_accepted = [this](const std::shared_ptr<GoalHandleDTP> goal_handle)
			{
				// thread off this function so that handle_accepted ends reasonably fast
				auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
				std::thread{execute_in_thread}.detach();
			};

			// create instantiates new action server
			this->action_server_ = rclcpp_action::create_server<DriveToPose>(
			  	this,
				"drive_to_pose",
				handle_goal,
				handle_cancel,
				handle_accepted);

			RCLCPP_INFO(get_logger(), "DriveToPose action server instantiated");
		}

	private:
		rclcpp_action::Server<DriveToPose>::SharedPtr action_server_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
		rclcpp::TimerBase::SharedPtr timer_;

		double current_x_ = 0.0;
		double current_y_ = 0.0;
		double current_yaw_ = 0.0;

		//declare PID controllers for each direction
		PID PID_x_;
		PID PID_y_;
		PID PID_yaw_;

		void execute(const std::shared_ptr<GoalHandleDTP> goal_handle)
		{
			RCLCPP_INFO(this->get_logger(), "Executing goal");
			rclcpp::Rate loop_rate(50); // 50 Hz

			auto result = std::make_shared<DriveToPose::Result>();
			auto feedback = std::make_shared<DriveToPose::Feedback>();

			auto prev_time = now();

			const auto goal = goal_handle->get_goal();
			double x_goal = goal->goal_pose.pose.position.x;
			double y_goal = goal->goal_pose.pose.position.y;

			const auto &q = goal->goal_pose.pose.orientation;

			double yaw_goal_raw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
			double yaw_goal = normalize_angle(yaw_goal_raw);
			while(rclcpp::ok())
			{
				//kill robot if we cancel the goal
				if (goal_handle->is_canceling())
				{
					kill_robot();
					goal_handle->canceled(result);
					return;
				}

				auto current_time = now();
				double dt = (current_time - prev_time).seconds();
				prev_time = current_time;

				double x_error = x_goal - current_x_;
				double y_error = y_goal - current_y_;
				double yaw_error = yaw_goal - current_yaw_;

				// if very close to pose goal, end action
				if(std::abs(x_error) < goal->position_tolerance &&
				   std::abs(y_error) < goal->position_tolerance &&
				   std::abs(yaw_error) < goal->yaw_tolerance)
				{
					kill_robot();
					result->success = true;
					goal_handle->succeed(result);
					return;
				}

				geometry_msgs::msg::Twist cmd_vel;
				cmd_vel.linear.x = PID_x_.compute(x_error, dt);
				cmd_vel.linear.y = PID_y_.compute(y_error, dt);
				cmd_vel.angular.z = PID_yaw_.compute(yaw_error, dt);

				cmd_vel_pub_->publish(cmd_vel);

				feedback->x_error = x_error;
				feedback->y_error = y_error;
				feedback->yaw_error = yaw_error;
				goal_handle->publish_feedback(feedback);

				loop_rate.sleep();
			}
		}

		// should probably also use the enable service to disable the wheels
		void kill_robot()
		{
			geometry_msgs::msg::Twist cmd_vel;
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.angular.z = 0;

			cmd_vel_pub_->publish(cmd_vel);
			// reset controllers
			PID_x_.reset();
			PID_y_.reset();
			PID_yaw_.reset();
		}

		double normalize_angle(double angle)
		{
			while(angle > M_PI) angle -= 2.0 * M_PI;
			while(angle < -M_PI) angle += 2.0 * M_PI;
			return angle;
		}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DriveToPoseServer>());
	rclcpp::shutdown();
	return 0;

}
