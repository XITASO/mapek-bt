#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "system_interfaces/msg/set_blackboard_group.hpp"


using namespace BT;


class Monitoring: public RosTopicSubNode<system_interfaces::msg::SetBlackboardGroup> 
{
public:
    Monitoring(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params)
        : RosTopicSubNode<system_interfaces::msg::SetBlackboardGroup>(name, conf, params), blackboard_(conf.blackboard)
        {
            node_handle = params.nh.lock();
            if (!node_handle) // Check if the weak pointer was successfully locked
            {
                throw RuntimeError("[Monitoring] Could not lock the node handle, will not be able to provide the time");
            }
        }

    NodeStatus onTick(const std::shared_ptr<system_interfaces::msg::SetBlackboardGroup>& msg) override;
    

    static PortsList providedPorts()
    {
        return {
            OutputPort<int>("clock")
        };
    }

private:
    Blackboard::Ptr blackboard_;

    std::shared_ptr<rclcpp::Node> node_handle;
};