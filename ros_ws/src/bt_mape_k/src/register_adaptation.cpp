#include "bt_mape_k/register_adaptation.hpp"
#include "bt_mape_k/util/string_utils.hpp"
#include "bt_mape_k/util/adaptation_utils.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

/**
 * @brief Constructor for the RegisterAdaptation class. This class implements a behavior tree
 *        node responsible for registering an adaptation action based on given inputs.
 * 
 * @param name A string specifying the name of the node.
 * @param config A NodeConfiguration object containing configuration parameters for the node.
 * 
 * @throws RuntimeError If rule_name input is not provided.
 * @throws std::runtime_error If required inputs for creating an adaptation are missing.
 * 
 * This constructor initializes the node's inputs, retrieving the type of adaptation and
 * parsing necessary parameters. It connects to the global experiment logger to log the
 * adaptation action for further evaluation or monitoring.
 */
RegisterAdaptation::RegisterAdaptation(const std::string& name, const NodeConfiguration& config)
: SyncActionNode(name, config){
    experiment_logger = BTLogger::get_global_logger();

    auto action = getInput<std::string>("action");
    auto component_name = getInput<std::string>("component_name");
    auto para_name = getInput<std::string>("parameter_name");
    auto value = getInput<std::string>("value");
    auto data_type = getInput<std::string>("data_type");
    auto bb_rule_name = getInput<std::string>("rule_name");

    if (!bb_rule_name)
    {
        throw RuntimeError("No rule name given:%s", bb_rule_name.error());
    }
    rule_name = bb_rule_name.value();
    message.rule_name = rule_name;
    message.source = "registerAdaptation_" + component_name.value();
    

    if (! (action && para_name && value && data_type))
        throw std::runtime_error("Cannot create adaptaion, missing input"); 

    auto adaptation_type = adaptations_utils::deduceAdaptationType(action.value());

    armed_adaptation.action = adaptation_type;

    if (   adaptation_type == system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER
        || adaptation_type == system_interfaces::msg::AdaptationType::ACTION_INCREASE_PARAMETER
        || adaptation_type == system_interfaces::msg::AdaptationType::ACTION_DECREASE_PARAMETER
        || adaptation_type == system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION
    ){
        auto data_type_value = data_type.value();
        if (data_type_value == "auto")
            data_type_value = deduceDataType(data_type_value);
        parseAdaptation(armed_adaptation, para_name.value(), value.value(), data_type_value);
    }
}
   
/**
 * @brief Executes the node's tick action, which registers an adaptation action, updates
 *        its inputs, and communicates the adaptations via the output port.
 * 
 * @return NodeStatus indicating the result of the action execution, returns NodeStatus::SUCCESS
 *         upon successful registration of the adaptation.
 * 
 * @throws RuntimeError If component_name input is not provided.
 * 
 * This method retrieves existing adaptations, appends the new adaptation action, logs the
 * adaptation event, and then updates the output with the modified adaptations list.
 */
BT::NodeStatus RegisterAdaptation::tick(){   
        auto existing_adaptations = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");
        std::vector<system_interfaces::msg::GenericAdaptation> adaptations;
        if (!existing_adaptations){
            adaptations = std::vector<system_interfaces::msg::GenericAdaptation>();
        }else{
            adaptations = existing_adaptations.value();
        }

        auto component_name = getInput<std::string>("component_name");
        if (!component_name){
            throw RuntimeError("Error getting input port [component_name]", component_name.error());
        }


        message.timestamp = experiment_logger->get_time(); 
        experiment_logger->silent_global(&message);

        adaptations.push_back(armed_adaptation);

        setOutput("adaptations", adaptations);

        return NodeStatus::SUCCESS;
}

/**
 * @brief Determines the appropriate data type of a given string value.
 * Currently supported: bool, double, int, string
 * 
 * @param value A string whose data type needs to be deduced.
 * @return A string representing the deduced data type (e.g., "bool", "double", "int", or "string").
 * 
 * This function utilizes type conversion attempts to deduce whether the input string can
 * be cast to known primitive types, returning the corresponding type as a string.
 */
std::string RegisterAdaptation::deduceDataType(std::string value){
    bool bv;
    double dv; 
    int iv;
    if (string_utils::to_bool(value, bv))
        return "bool";
    if (string_utils::to_double(value, dv))
        return "double";
    if (string_utils::to_int(value, iv))
        return "int";
    return "string";
}

/**
 * @brief Parses adaptation details and assigns values to the adaptation object based on the parameter type.
 * 
 * @param adaptation A GenericAdaptation object to populate with parsed information.
 * @param para_name The name of the parameter to be adapted.
 * @param value The value to set for the parameter.
 * @param datatype The data type of the parameter, which dictates how the value is assigned.
 * 
 * @throws std::runtime_error If an unsupported data type is specified.
 * 
 * This method assigns the parameter name and its value to the GenericAdaptation (specified by system_interfaces/GenericAdaptation) 
 * object according to its specified data type, ensuring correct parameter type assignment in the ROS2 environment.
 */
void RegisterAdaptation::parseAdaptation(system_interfaces::msg::GenericAdaptation& adaptation, const std::string& para_name, const std::string& value, const std::string& datatype){
    adaptation.parameter.name = para_name;

    if (datatype == "bool"){
        adaptation.parameter.value.bool_value = string_utils::to_bool(value);
        adaptation.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    }else if (datatype == "string"){
        adaptation.parameter.value.string_value = value;
        adaptation.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    }else if (datatype == "double"){
        adaptation.parameter.value.bool_value = std::stof(value);
        adaptation.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    }else if (datatype == "int"){
        adaptation.parameter.value.bool_value = std::stoi(value);
        adaptation.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    }
    else{
        throw std::runtime_error("unsupported datatype: " + datatype);
    }
}

