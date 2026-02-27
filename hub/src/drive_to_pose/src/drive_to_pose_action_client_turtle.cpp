#include <chrono>

#include "dtp_interfaces/action/drive_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class DriveToPoseClient : public rclcpp::Node
{
	public:
		using DriveToPose = dtp_interfaces::action::DriveToPose;
		using GoalHandleDTP = rclcpp_action::ClientGoalHandle<DriveToPose>;

		// class constructor to initialize node
		explicit DriveToPoseClient(const rclcpp::NodeOptions & options)
		: Node("drive_to_pose_action_client", options)
		{
			//instantiates new action client
			this->client_ptr_ = rclcpp_action::create_client<DriveToPose>(this, "drive_to_pose");

			// create timer callback to get pose
			goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
				"/goal_pose",
				10,
				[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
				{
					this->send_goal(*msg, 0.01, 0.01);
				});
			RCLCPP_INFO(this->get_logger(), "Action client was instantiated");
		}

		void send_goal(const geometry_msgs::msg::PoseStamped & pose, double pos_tol, double yaw_tol)
		{
			if(!this->client_ptr_->wait_for_action_server())
			{
				RCLCPP_ERROR(this->get_logger(), "Action server not available after timeout");
				rclcpp::shutdown();
			}

			DriveToPose::Goal goal_msg;
			goal_msg.goal_pose = pose;
			goal_msg.position_tolerance = pos_tol;
			goal_msg.yaw_tolerance = yaw_tol;

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
						RCLCPP_INFO(this->get_logger(), "Reached pose goal!");
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
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DriveToPoseClient>(rclcpp::NodeOptions()));
	rclcpp::shutdown();
	return 0;
}
