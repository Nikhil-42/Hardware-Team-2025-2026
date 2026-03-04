#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "hub_interfaces/action/start.hpp"

using namespace BT;
using Start = hub_interfaces::action::Start;

class StartNode : public RosActionNode<Start>
{
public:
    StartNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params) : RosActionNode<Start>(name, conf, params) {}
    
    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("action_name", "/start_light"),
        });
    }

    bool setGoal(Goal& goal) override;
    void onHalt() override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
};