#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include "hub_interfaces/srv/gate.hpp"

using namespace BT;
using Gate = hub_interfaces::srv::Gate;

class GateNode : public RosServiceNode<Gate>
{
public:
	explicit GateNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
				: RosServiceNode<Gate>(name, conf, params) {}

	static PortsList providedPorts()
	{
		return providedBasicPorts({
			InputPort<std::string>("server_name", "/gate_node"),
			InputPort<bool>("open")
		});
	}

	bool setRequest(Request::SharedPtr& request) override;
	NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
	virtual NodeStatus onFailure(ServiceNodeErrorCode errror) override;
};
