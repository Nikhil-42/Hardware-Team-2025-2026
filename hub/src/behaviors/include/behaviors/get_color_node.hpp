#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include "hub_interfaces/action/get_color.hpp"

using namespace BT;
using GetColor = hub_interfaces::action::GetColor;

class GetColorNode : public RosActionNode<GetColor>
{
public:
	GetColorNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosActionNode<GetColor>(name, conf, params) {}

	static PortsList providedPorts()
	{
		return providedBasicPorts({
			InputPort<std::string>("action_name", "/drive_with_vel"),
			OutputPort<uint8_t>("color")
		});
	}

	bool setGoal(Goal& goal) override;
	void onHalt() override;
	NodeStatus onResultReceived(const WrappedResult& wr) override;
	virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
};
