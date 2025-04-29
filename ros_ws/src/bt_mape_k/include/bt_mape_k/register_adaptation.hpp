#include "behaviortree_cpp/bt_factory.h"
#include "bt_mape_k/util/logger.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "system_interfaces/msg/experiment_logging.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace BT;

// Planning and registering adaptations for general rules
class RegisterAdaptation: public SyncActionNode
{
public:
    RegisterAdaptation(const std::string& name, const NodeConfiguration& config);

    static PortsList providedPorts()
    {
        return { 
            InputPort<std::string>("component_name"),
            InputPort<std::string>("rule_name"),
            InputPort<std::string>("action"),
            InputPort<std::string>("parameter_name"),
            InputPort<std::string>("value"),
            InputPort<std::string>("data_type", "auto", "datatype, if auto is set the system tries its best to cast it accordingly"),
            BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations")
         };
    }

    NodeStatus tick() override;

    private: 
        std::string deduceDataType(std::string);
        void parseAdaptation(system_interfaces::msg::GenericAdaptation&, const std::string& para_name, const std::string& value, const std::string& datatype);
        system_interfaces::msg::GenericAdaptation armed_adaptation;
        std::string rule_name;

        std::shared_ptr<BTLogger> experiment_logger;
        system_interfaces::msg::ExperimentLogging message;
    
};