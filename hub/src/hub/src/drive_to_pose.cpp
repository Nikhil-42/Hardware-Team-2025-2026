#include <thread>
#include <cmath>


#include "hub_interfaces/action/drive_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "hub/pid.hpp"

#include <tf2/utils.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DriveToPoseServer : public rclcpp::Node
{

	using DriveToPose = hub_interfaces::action::DriveToPose;
	using GoalHandleDTP = rclcpp_action::ServerGoalHandle<DriveToPose>;

	public:
		// velocity clamps: vx = 0.16, vy = 0.18, wz = 1.0
		// constructor for class that initializes the node as the action server
		explicit DriveToPoseServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
		  : Node("drive_to_pose_action_server", options),
		    PID_x_(1.5, 0.0, 0.0, 0.30),
		    PID_y_(1.5, 0.0, 0.0, 0.30),
		    PID_yaw_(1.5, 0.0, 0.0, 1.5)
		{
			cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

			// initialize tf2 buffer and listener for map -> base_link lookups
			tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
			tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
			// pose is polled in execute so no need for odom sub

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
		rclcpp::TimerBase::SharedPtr timer_;

		std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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
			rclcpp::Rate loop_rate(500); // 50 Hz

			auto result = std::make_shared<DriveToPose::Result>();
			auto feedback = std::make_shared<DriveToPose::Feedback>();

			auto prev_time = now();

			const auto goal = goal_handle->get_goal();
			double x_goal = goal->pose.position.x;
			double y_goal = goal->pose.position.y;

			const auto &q = goal->pose.orientation;

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

				// look up current pose from map -> base_link TF transform
				try
				{
					// base link's pose in map frame. Need to transform to base link
					auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
					current_x_ = transform.transform.translation.x;
					current_y_ = transform.transform.translation.y;
					current_yaw_ = tf2::getYaw(transform.transform.rotation);
				}
				catch (const tf2::TransformException &ex)
				{
					RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
					loop_rate.sleep();
					continue;
				}

				auto current_time = now();
				double dt = (current_time - prev_time).seconds();
				prev_time = current_time;

				double x_error = x_goal - current_x_;
				double y_error = y_goal - current_y_;
				double yaw_error = normalize_angle(yaw_goal - current_yaw_);

				double distance_error = std::sqrt(x_error * x_error + y_error * y_error);

				RCLCPP_DEBUG(this->get_logger(), "x_error: %f y_error: %f yaw_error: %f", x_error, y_error, yaw_error);
				// if very close to pose goal, end action
				if(distance_error < goal->position_tolerance &&
				   std::abs(yaw_error) < goal->yaw_tolerance)
				{
					kill_robot();
					result->success = true;
					goal_handle->succeed(result);
					return;
				}
				

				geometry_msgs::msg::Twist cmd_vel;
				double cmd_x_map = PID_x_.compute(x_error, dt);
				double cmd_y_map = PID_y_.compute(y_error, dt);
				double cmd_yaw_map = PID_yaw_.compute(yaw_error, dt);

				// transform cmd_vel from map frame to base_link frame
				cmd_vel.linear.x = std::cos(current_yaw_) * cmd_x_map + std::sin(current_yaw_) * cmd_y_map;
				cmd_vel.linear.y = -std::sin(current_yaw_) * cmd_x_map + std::cos(current_yaw_) * cmd_y_map;
				cmd_vel.angular.z = cmd_yaw_map;

				cmd_vel_pub_->publish(cmd_vel);
				//RCLCPP_INFO(this->get_logger(), "Published /cmd_vel from server");
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

			for(int i = 0; i < 5; i++)
			{
				cmd_vel_pub_->publish(cmd_vel);
				rclcpp::sleep_for(std::chrono::milliseconds(10));
			}

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
