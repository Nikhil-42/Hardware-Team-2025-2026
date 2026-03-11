#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "hub_interfaces/action/drive_with_vel.hpp"

using namespace BT;
using DriveWithVel = hub_interfaces::action::DriveWithVel;

class DriveWithVelNode : public RosActionNode<DriveWithVel>
{
public:
	DriveWithVelNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosActionNode<DriveWithVel>(name, conf, params) {}

	static PortsList providedPorts()
	{
		return providedBasicPorts({
			InputPort<std::string>("action_name", "/drive_with_vel"),
			InputPort<double>("vx"),
			InputPort<double>("vy"),
			InputPort<double>("wz")
		});
	}

	bool setGoal(Goal& goal) override;
	void onHalt() override;
	NodeStatus onResultReceived(const WrappedResult& wr) override;
	virtual NodeStatus onFailure(ActionnodeErrorCode error) override;
}
