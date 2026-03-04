#include <thread>

#include "hub_interfaces/action/drive_with_vel.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

			cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

			auto odom_callback = [this](nav_msgs::msg::Odometry::SharedPtr msg) ->void
			{
				current_vel_ = msg->twist.twist;
			};

			odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10, odom_callback);

			auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DriveWithVel::Goal> goal)
			{
				RCLCPP_INFO(this->get_logger(), "Received goal request");
				(void)uuid;
				(void)goal;
				return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
			};

			auto handle_cancel = [this](const std::shared_ptr<GoalHandleDWV> goal_handle)
			{
				RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
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
				handle_goal,
				handle_cancel,
				handle_accepted);

			RCLCPP_INFO(get_logger(), "DriveWithVel action server instantiated");
		}

	private:
		rclcpp_action::Server<DriveWithVel>::SharedPtr action_server_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

		geometry_msgs::msg::Twist current_vel_;
		double max_duration = 5.0;

		void execute(const std::shared_ptr<GoalHandleDWV> goal_handle)
		{
			RCLCPP_INFO(this->get_logger(), "Executing goal");
			rclcpp::Rate loop_rate(500);

			auto feedback = std::make_shared<DriveWithVel::Feedback>();
			auto result = std::make_shared<DriveWithVel::Result>();
			auto start_time = now();

			const auto goal = goal_handle->get_goal();

			geometry_msgs::msg::Twist cmd_vel = goal->cmd_vel;
			while(rclcpp::ok())
			{

				goal_handle->publish_feedback(feedback);

				if(goal_handle->is_canceling())
				{
					kill_robot();
					result->success = false;
					goal_handle->canceled(result);
					return;
				}

				cmd_vel_pub_->publish(cmd_vel);

				feedback->current_vel = current_vel_;
				goal_handle->publish_feedback(feedback);
				loop_rate.sleep();

				if ((now() - start_time).seconds() > max_duration)
				{
				    kill_robot();
				    result->success = true;
				    goal_handle->succeed(result);
				    return;
				}
			}
		}

		void kill_robot()
		{
			geometry_msgs::msg::Twist cmd_vel;
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.angular.z = 0;

			for(int i = 0; i < 5; i++)
			{
				cmd_vel_pub_->publish(cmd_vel);
				rclcpp::sleep_for(std::chrono::milliseconds(10));
			}
		}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DriveWithVelServer>());
	rclcpp::shutdown();
	return 0;
}
