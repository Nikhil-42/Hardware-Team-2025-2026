#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include "hub_interfaces/action/motor.hpp"

using namespace BT;
using Motor = hub_interfaces::action::Motor;

class MotorNode : public RosActionNode<Motor>
{
public:
	MotorNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosActionNode<Motor>(name, conf, params) {}

	static PortsList providedPorts()
	{
		return providedBasicPorts({
			InputPort<std::string>("velocity", "1.0"),
		});
	}

	bool setGoal(Goal& goal) override;
	void onHalt() override;
	NodeStatus onResultReceived(const WrappedResult& wr) override;
	virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
};
