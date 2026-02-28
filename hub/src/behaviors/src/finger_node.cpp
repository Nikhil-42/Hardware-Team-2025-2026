#include "behaviors/finger_node.hpp"
#include <string>

bool FingerNode::setRequest(Request::SharedPtr& request)
{
  getInput("idx", request->idx);
  std::cout << "setRequest " << std::endl;
  return true;
}

BT::NodeStatus FingerNode::onResponseReceived(const Response::SharedPtr& response)
{
  std::cout << "onResponseReceived " << std::endl;
  if(response->success)
  {
    RCLCPP_INFO(logger(), "Finger service succeeded.");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(logger(), "Finger service failed.");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus FingerNode::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}
