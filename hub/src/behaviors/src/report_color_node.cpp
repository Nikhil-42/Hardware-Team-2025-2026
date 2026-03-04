#include "behaviors/report_color_node.hpp"
#include <string>

using namespace BT;

bool ReportColorNode::setRequest(Request::SharedPtr& request)
{
  getInput("antenna", request->antenna);
  getInput("color", request->color);
  std::cout << "setRequest " << std::endl;
  return true;
}

NodeStatus ReportColorNode::onResponseReceived(const Response::SharedPtr& response)
{
  std::cout << "onResponseReceived " << std::endl;
  if(response->success)
  {
    RCLCPP_INFO(logger(), "ReportColor service succeeded.");
    return NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(logger(), "ReportColor service failed.");
    return NodeStatus::FAILURE;
  }
}

NodeStatus ReportColorNode::onFailure(ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s (%d)",
               name().c_str(), toStr(error), static_cast<int>(error));
  return NodeStatus::FAILURE;
}

