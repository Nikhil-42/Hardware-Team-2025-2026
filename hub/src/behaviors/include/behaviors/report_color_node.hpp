#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include "hub_interfaces/srv/report_color.hpp"

using ReportColor = hub_interfaces::srv::ReportColor;

class ReportColorNode : public RosServiceNode<ReportColor>
{
public:
  explicit ReportColorNode(const std::string& name, const NodeConfig& conf,
                          const RosNodeParams& params)
    : RosServiceNode<ReportColor>(name, conf, params)
  {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ InputPort<std::string>("server_name", "/click"), InputPort<uint8_t>("antenna"), InputPort<uint8_t>("color") });
  }

  bool setRequest(Request::SharedPtr& request) override;
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};
