#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include "hub_interfaces/srv/finger.hpp"

using Finger = hub_interfaces::srv::Finger;

class FingerNode : public BT::RosServiceNode<Finger>
{
public:
  explicit FingerNode(const std::string& name, const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params)
    : RosServiceNode<Finger>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>("server_name", "/click"), BT::InputPort<int64_t>("idx"), BT::InputPort<std::string>("action") });
  }

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};
