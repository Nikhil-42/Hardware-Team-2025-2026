#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav2_behavior_tree/bt_action_node.hpp"

using namespace BT;

class NavigateToPoseBT : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
        NavigateToPoseBT(const std:: string& xml_tag_name,
			const std::string& action_name,
                        const NodeConfiguration& config)
	: nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, config) {}

        // takes a pose input as the goal
        static PortsList providedPorts()
        {
                return providedBasicPorts({InputPort<geometry_msgs::msg::PoseStamped>("goal")});
        }

	void on_tick() override
	{
		geometry_msgs::msg::PoseStamped goal;

		if(!getInput("goal", goal))
		{
			throw RuntimeError("Missing required input");
		}

		goal_.pose = goal;
	}

	NodeStatus on_success() override
	{
		RCLCPP_INFO(node_->get_logger(), "Navigation SUCCEEDED");
		return NodeStatus::SUCCESS;
	}

	NodeStatus on_aborted() override
  	{
    		RCLCPP_WARN(node_->get_logger(), "Navigation ABORTED");
    		return NodeStatus::FAILURE;
  	}

  	NodeStatus on_cancelled() override
 	{
    		RCLCPP_WARN(node_->get_logger(), "Navigation CANCELLED");
    		return NodeStatus::FAILURE;
	}
};
