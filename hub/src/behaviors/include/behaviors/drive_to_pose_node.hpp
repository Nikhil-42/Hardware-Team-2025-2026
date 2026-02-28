#include "behaviortree_ros2/bt_action_node.hpp"
#include "hub_interfaces/action/drive_to_pose.hpp"

using namespace BT;
using DriveToPose = hub_interfaces::action::DriveToPose;

class DriveToPoseNode : public RosActionNode<DriveToPose>
{
public:
    DriveToPoseNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosActionNode<DriveToPose>(name, conf, params) {}
    
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({ InputPort<std::string>("server_name", "/drive_to_pose"), InputPort<double>("x"), InputPort<double>("y"), InputPort<double>("yaw") });
    }

    bool setGoal(Goal& goal) override;
    void onHalt() override;
    BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};