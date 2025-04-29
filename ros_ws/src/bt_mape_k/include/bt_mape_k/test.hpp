#include <behaviortree_ros2/bt_service_node.hpp>
#include "rcl_interfaces/srv/set_parameters.hpp"

using SetParam = rcl_interfaces::srv::SetParameters;

class SetBoolService : public BT::RosServiceNode<SetParam>
{
public:
  explicit SetBoolService(const std::string& name, const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params)
    : RosServiceNode<SetParam>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("value") };
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string service_suffix_;
};