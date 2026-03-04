#include "behaviors/start_node.hpp"

using namespace BT;

bool StartNode::setGoal(RosActionNode::Goal& goal)
{
  return true;
}

NodeStatus StartNode::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? "true" : "false");

  return (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus StartNode::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void StartNode::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}
