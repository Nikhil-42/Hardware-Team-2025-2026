#include <chrono>

#include "hub_interfaces/action/drive_to_pose.hpp"
#include "rclcpp/rclpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DriveToPoseClient : public rclcpp::Node
{
	public:
		using DriveToPose = hub_interfaces::action::DriveToPose;
		using GoalHandleDTP = rclcpp_action::ClientGoalHandle<DriveToPose>;

		// class constructor to initialize node
		explicit DriveToPoseClient(const rclcpp::NodeOptions & options)
		: Node("drive_to_pose_action_client", options)
		{
			//instantiates new action client
			this->client_ptr_ = rclcpp_action::create_client<DriveToPose>(this, "drive_to_pose");

			// create timer callback to get pose
			auto timer_callback_lambda = [this](){return this->send_goal_pose(); };
			this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback_lambda);
		}

		void send_goal()
		{
			this->timer_->cancel();

			if(!this->client_ptr_->wait_for_action_server())
			{
				RCLCPP_ERROR(this->get_logger(), "Action server not available after timeout");
				rclcpp::shutdown();
			}

			auto goal::msg =

		}
}

