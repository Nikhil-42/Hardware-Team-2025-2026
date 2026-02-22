#include <chrono>

<<<<<<< HEAD
#include "dtp_interfaces/action/drive_to_pose.hpp"
=======
#include "hub_interfaces/action/drive_to_pose.hpp"
>>>>>>> f85e9d60a3f9b9e54e0df42ecb0a464a98d85ba5
#include "rclcpp/rclpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DriveToPoseClient : public rclcpp::Node
{
	public:
<<<<<<< HEAD
		using DriveToPose = dtp_interfaces::action::DriveToPose;
=======
		using DriveToPose = hub_interfaces::action::DriveToPose;
>>>>>>> f85e9d60a3f9b9e54e0df42ecb0a464a98d85ba5
		using GoalHandleDTP = rclcpp_action::ClientGoalHandle<DriveToPose>;

		// class constructor to initialize node
		explicit DriveToPoseClient(const rclcpp::NodeOptions & options)
		: Node("drive_to_pose_action_client", options)
		{
			//instantiates new action client
			this->client_ptr_ = rclcpp_action::create_client<DriveToPose>(this, "drive_to_pose");

			// create timer callback to get pose
<<<<<<< HEAD
			auto timer_callback_lambda = [this](){return this->send_goal(); };
			this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback_lambda);
		}

		void send_goal(const geometry_msgs::msg::PoseStamped & pose, double pos_tol, double yaw_tol)
=======
			auto timer_callback_lambda = [this](){return this->send_goal_pose(); };
			this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback_lambda);
		}

		void send_goal()
>>>>>>> f85e9d60a3f9b9e54e0df42ecb0a464a98d85ba5
		{
			this->timer_->cancel();

			if(!this->client_ptr_->wait_for_action_server())
			{
				RCLCPP_ERROR(this->get_logger(), "Action server not available after timeout");
				rclcpp::shutdown();
			}

<<<<<<< HEAD
			DriveToPose::Goal goal;
			goal.target_pose = pose;
			goal.position_tolerance = pos_tol;
			goal.yaw_tolerance = yaw_tol;

			RCLCPP_INFO(this->get_logger(), "Sending goal");

			auto send_goal_options = rclcpp_action::Client<DriveToPose>::SendGoalOptions();
			send_goal_options.goal_response_callback = [this](const GoalHandleDTP::SharedPtr & goal_handle)
			{
				if(!goal_handle)
				{
					RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
				}
				else
				{
					RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
				}

			};

			send_goal_options.feedback_callback = [this](GoalHandleDTP::SharedPtr, const std::shared_ptr<const DriveToPose::Feedback> feedback)
			{
				RCLCPP_INFO(this->get_logger(), "Remaining x %f", feedback->x_error);
				RCLCPP_INFO(this->get_logger(), "Remaining y %f", feedback->y_error);
				RCLCPP_INFO(this->get_logger(), "Remaining yaw %f", feedback->yaw_error);
			};

			send_goal_options.result_callback = [this](const GoalHandleDTP::WrappedResult &result)
			{
				switch(result.code)
				{
					case rclcpp_action::ResultCode::SUCCEEDED:
						break;
					case rclcpp_action::ResultCode::ABORTED:
						RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
						return;
					case rclcpp_action::ResultCode::CANCELED:
						RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
						return;
					default:
						RCLCPP_ERROR(this->get_logger(), "Unknown result code");
						return;
				}
				rclcpp::shutdown();
			};
			this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
		}
	private:
		rclcpp_action::Client<DriveToPose>::SharedPtr client_ptr_;
		rclcpp::TimerBase::SharedPtr timer_;
};
=======
			auto goal::msg =

		}
}
>>>>>>> f85e9d60a3f9b9e54e0df42ecb0a464a98d85ba5

