#include "behaviors/gate_node.hpp"

using namespace BT;

bool GateNode::setRequest(Request::SharedPtr& request)
{
	getInput("open", request->open);
	return true;
}

NodeStatus GateNode::onResponseReceived(const Response::SharedPtr& response)
{
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

NodeStatus GateNode::onFailure(ServiceNodeErrorCode error)
{
	RCLCPP_ERROR(logger(), "%s: onFailure with error: %s (%d)",
				name().c_str(), toStr(error), static_cast<int>(error));
	return NodeStatus::FAILURE;
}

