#include "behaviors/get_color_node.hpp"

using namespace BT;

bool GetColorNode::setGoal(RosActionNode::Goal& goal)
{
	return true;
}

NodeStatus GetColorNode::onResultReceived(const RosActionNode::WrappedResult& wr)
{
	setOutput("color", wr.result->color);

	RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
				(wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? "true" : "false");

	return (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus GetColorNode::onFailure(ActionNodeErrorCode error)
{
	RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
	return NodeStatus::FAILURE;
}

void GetColorNode::onHalt()
{
	RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}
