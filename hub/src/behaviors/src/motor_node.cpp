#include "behaviors/motor_node.hpp"

using namespace BT;

bool MotorNode::setGoal(RosActionNode::Goal& goal)
{
    getInput("velocity", goal.velocity);
    std::cout << "setRequest " << std::endl;
	return true;
}

NodeStatus MotorNode::onResultReceived(const RosActionNode::WrappedResult& wr)
{
	RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
				(wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? "true" : "false");

	return (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus MotorNode::onFailure(ActionNodeErrorCode error)
{
	RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
	return NodeStatus::FAILURE;
}

void MotorNode::onHalt()
{
	RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}
