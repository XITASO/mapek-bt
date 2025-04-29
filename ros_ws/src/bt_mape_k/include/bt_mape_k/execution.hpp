#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/bt_service_node.hpp>

#include "bt_mape_k/util/adaptation_utils.hpp"
#include "bt_mape_k/util/logger.hpp"

#include "system_interfaces/msg/comm_change_input.hpp"
#include "system_interfaces/msg/parametrization_input.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "system_interfaces/srv/set_lifecycle_changes.hpp"
#include <system_interfaces/msg/generic_adaptation.hpp>
#include <system_interfaces/msg/adaptation_type.hpp>
#include <system_interfaces/msg/adaptation_status.hpp>
#include <system_interfaces/msg/experiment_logging.hpp>
#include <cstdlib> 
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include "bt_mape_k/util/string_utils.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace BT;
using AdaptationType = system_interfaces::msg::AdaptationType;

class ExecuteParametrization: public RosServiceNode<rcl_interfaces::srv::SetParameters>
{
public:
    ExecuteParametrization(const std::string& name, const NodeConfiguration& config, const RosNodeParams& params);

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("component_name"),
                BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations"),
                BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations")
         };
    }

    bool setRequest(Request::SharedPtr& request) override;

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    NodeStatus onFailure(ServiceNodeErrorCode error) override;

    NodeStatus tick() override;

    bool ParseParameterUpdateResponse(std::vector<rcl_interfaces::msg::SetParametersResult>& Result);

    private:
        // Member variable for the publisher
        std::shared_ptr<BTLogger> experiment_logger;
        system_interfaces::msg::ExperimentLogging message;

        std::vector<uint8_t> relevantAdaptationTypes;
        std::vector<system_interfaces::msg::GenericAdaptation> current_adaptations {};

        void logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success);
        
};

class ExecuteRedeploy : public RosServiceNode<lifecycle_msgs::srv::ChangeState>
{
public:
    ExecuteRedeploy(const std::string& name, const NodeConfiguration& config, const RosNodeParams& params);

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("component_name"),
            BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations"),
            BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations")
        };
    }

    bool setRequest(Request::SharedPtr& request) override;

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    NodeStatus onFailure(ServiceNodeErrorCode error) override;

    NodeStatus tick() override;
   
private:
    std::shared_ptr<rclcpp::Node> node_handle;
    std::shared_ptr<BTLogger> experiment_logger;
    system_interfaces::msg::ExperimentLogging message;
    std::string component_name;
    std::string executable_name;
    std::string package_name;

    std::vector<uint8_t> relevantAdaptationTypes;

    // Kill process by name
    bool kill_process_by_name(const std::string& exec_name);

    // Restart using a user-provided command
    bool restart_with_command(const std::string& command);

    bool restart_ros2_node(const std::string& package, const std::string& executable);

    void logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success);
};

class ExecuteCommChange: public RosServiceNode<rcl_interfaces::srv::SetParameters>
{
public:
    ExecuteCommChange(const std::string& name, const NodeConfiguration& config, const RosNodeParams& params);

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("component_name"),
                BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations"),
                BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations")
         };
    }

    bool setRequest(Request::SharedPtr& request) override;

    bool ParseParameterUpdateResponse(std::vector<rcl_interfaces::msg::SetParametersResult>& Result);

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    NodeStatus onFailure(ServiceNodeErrorCode error) override;

    NodeStatus tick() override;

    private:
        // Member variable for the publisher
        std::shared_ptr<BTLogger> experiment_logger;
        system_interfaces::msg::ExperimentLogging message;

        std::vector<uint8_t> relevantAdaptationTypes;
        std::vector<system_interfaces::msg::GenericAdaptation> current_adaptations {};

        void logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success);
};

class ExecuteLifeCycle: public RosServiceNode<system_interfaces::srv::SetLifecycleChanges>
{
public:
    ExecuteLifeCycle(const std::string& name, const NodeConfiguration& config, const RosNodeParams& params);

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("component_name"),
                BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations"),
                BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations"),
                OutputPort<int>("lc_state")
         };
    }

    bool setRequest(Request::SharedPtr& request) override;

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    NodeStatus onFailure(ServiceNodeErrorCode error) override;

    NodeStatus tick() override;

    private:
        std::shared_ptr<BTLogger> experiment_logger;
        system_interfaces::msg::ExperimentLogging message;
        std::string component_name;

        std::vector<uint8_t> relevantAdaptationTypes;
        std::vector<system_interfaces::msg::GenericAdaptation> current_adaptations {};

        void logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success);
};

