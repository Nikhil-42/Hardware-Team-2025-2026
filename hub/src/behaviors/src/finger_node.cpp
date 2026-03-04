#include "behaviors/finger_node.hpp"
#include <string>

using namespace BT;

bool FingerNode::setRequest(Request::SharedPtr& request)
{
  getInput("idx", request->idx);
  std::cout << "setRequest " << std::endl;
  return true;
}

NodeStatus FingerNode::onResponseReceived(const Response::SharedPtr& response)
{
  std::cout << "onResponseReceived " << std::endl;
  if(response->success)
  {
    RCLCPP_INFO(logger(), "Finger service succeeded.");
    return NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(logger(), "Finger service failed.");
    return NodeStatus::FAILURE;
  }
}

NodeStatus FingerNode::onFailure(ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s (%d)",
               name().c_str(), toStr(error), static_cast<int>(error));
  return NodeStatus::FAILURE;
}

