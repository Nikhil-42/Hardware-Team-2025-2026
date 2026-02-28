#include "behaviors/drive_to_pose_node.hpp"

const double PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664709384460955058223172535940812848111745028410270193852110555964462294895493038196442881097566593344612847564823378678316527120190914564856692346034861045432664821339360726024914127372458700660631558817488152092096282925409171536436789259036001133053054882046652138414695194151160943305727036575959195309218611738193261179310511854807446237996274956735188575272489122793818301194912983367336244065664308602139494639522473719070217986094370277053921717629317675238467481846766940513200;

bool DriveToPoseNode::setGoal(RosActionNode::Goal& goal)
{
  goal.pose.position.x = getInput<double>("x").value();
  goal.pose.position.y = getInput<double>("y").value();
  double yaw = getInput<double>("yaw").value() / 180 * PI;

  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = std::sin(yaw / 2);
  goal.pose.orientation.w = std::cos(yaw / 2);

  return true;
}

NodeStatus DriveToPoseNode::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->success ? "true" : "false");

  return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus DriveToPoseNode::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void DriveToPoseNode::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}
