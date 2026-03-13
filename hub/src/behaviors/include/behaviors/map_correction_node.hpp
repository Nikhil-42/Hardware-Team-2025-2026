#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "hub_interfaces/srv/set_map_pose.hpp"

class MapCorrectionNode : public BT::RosServiceNode<hub_interfaces::srv::SetMapPose>
{
public:
    using Request  = hub_interfaces::srv::SetMapPose::Request;
    using Response = hub_interfaces::srv::SetMapPose::Response;

    MapCorrectionNode(
        const std::string & instance_name,
        const BT::NodeConfig & conf,
        const BT::RosNodeParams & params)
      : BT::RosServiceNode<hub_interfaces::srv::SetMapPose>(instance_name, conf, params)
    {}

    // Declare the input ports this node expects in the BT XML
    static BT::PortsList providedPorts()
    {
        return {
            // Known map-frame position of the landmark the robot has reached
            BT::InputPort<double>("known_x",   "Known x position in map frame"),
            BT::InputPort<double>("known_y",   "Known y position in map frame"),
            BT::InputPort<double>("known_yaw", "Known yaw in map frame (radians)")
        };
    }

    bool setRequest(Request::SharedPtr & request) override;
    BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
