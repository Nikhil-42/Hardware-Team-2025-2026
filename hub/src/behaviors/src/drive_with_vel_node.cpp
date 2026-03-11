#include "behaviors/drive_with_vel_node.cpp"

bool DriveWithVelNode::setGoal(RosActionNode::Goal& goal)
{
	goal.linear.x = getInput<double>("vx").value();
	goal.linear.y = getInput<double>("vy").value();
	goal.twist.z = getInput<double>("wz").value();

	return true;
}

NodeStatus DriveWithVelNode::onResultReceived(const RosActionNode::WrappedResult& wr)
{
	RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
				wr.result->success ? "true" : "false");
	return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus DriveWithVelNode::onFailure(ActionNodeErrorCode error)
{
	RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
	return NodeStatus::FAILURE;
}

void DriveWithVel::onHalt()
{
	RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}
