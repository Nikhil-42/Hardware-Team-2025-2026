#include "behaviortree_ros2/plugins.hpp"

#include "behaviors/drive_to_pose_node.hpp"
#include "behaviors/finger_node.hpp"

extern "C" void BT_RegisterRosNodeFromPlugin(
  BT::BehaviorTreeFactory& factory,
  const BT::RosNodeParams& params)
{
  // This is the part that depends on your BT ROS2 wrapper version.
  // In many setups, this works:
  factory.registerNodeType<DriveToPoseNode>("DriveToPose", params);
  factory.registerNodeType<FingerNode>("Finger", params);

  // If the above doesn't compile, the wrapper might provide a helper instead.
  // In that case, tell me what compile error you get and I’ll adapt it.
}