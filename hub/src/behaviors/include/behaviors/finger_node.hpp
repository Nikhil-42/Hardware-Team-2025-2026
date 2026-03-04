#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include "hub_interfaces/srv/finger.hpp"

using namespace BT;
using Finger = hub_interfaces::srv::Finger;

class FingerNode : public RosServiceNode<Finger>
{
public:
  explicit FingerNode(const std::string& name, const NodeConfig& conf,
                          const RosNodeParams& params)
    : RosServiceNode<Finger>(name, conf, params)
  {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ InputPort<std::string>("server_name", "/click"), InputPort<int64_t>("idx"), InputPort<std::string>("action") });
  }

  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};
