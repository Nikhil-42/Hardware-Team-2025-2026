#include <thread>

#include "hub_interfaces/action/drive_with_vel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DriveWithVelServer : public rclcpp::Node
{
	public:
		using DriveWithVel = hub_interfaces::action::DriveWithVel;
		using GoalHandleDWV = rclcpp_action::ServerGoalHandle<DriveWithVel>;

		explicit DriveWithVelServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
		: Node ("drive_with_vel_action_server", options)
		{
			auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DriveWithVel> goal)
			{
				RCLCPP_INFO(this->get_logger(), "Received goal request");
				(void)uuid;
				return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
			};

			auto handle_cancel = [this](const std::shared_ptr<DriveWithVel> goal_handle)
			{
				RCLCPP_INFO(this->get_logger(), "Received requrest to cancel goal");
				(void)goal_handle;
				kill_robot();
				return rclcpp_action::CancelResponse::ACCEPT;
			};

			auto handle_accepted = [this](const std::shared_ptr<GoalHandleDWV> goal_handle)
			{
				//creates new thread to avoid block executor when in this lambda function
				auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
				std::thread{execute_in_thread}.detach();
			};

			this->action_server_ = rclcpp_action::create_server<DriveWithVel>(
				this,
				"drive_with_vel",
				
		}

	private:
		rclcpp_action:Server<DriveWithVel>::SharedPtr action_server_;

}

