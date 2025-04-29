#include "bt_mape_k/execution.hpp"
#include "bt_mape_k/util/adaptation_utils.hpp"
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
using AdaptationType = system_interfaces::msg::AdaptationType;

/**
 * @brief Constructor for the ExecuteCommChange class
 * @param name The name of the node.
 * @param config Configuration for the node.
 * @param params Parameters required for ROS node setup.
 *
 * Initializes the service name, logger for experiment status, and adaptation types relevant to communication changes.
 */
ExecuteCommChange::ExecuteCommChange(const std::string& name, const NodeConfiguration& config, const RosNodeParams& params) : 
RosServiceNode<rcl_interfaces::srv::SetParameters>(name, config, params)
{
    auto component_res = getInput<std::string>("component_name");
    if (!component_res)
    {
        throw RuntimeError("Error getting input port [component_name]", component_res.error());
    }
    std::string component_name = component_res.value();

    // Initialize the random publisher
    experiment_logger = BTLogger::get_global_logger();
    message = system_interfaces::msg::ExperimentLogging();
    message.source = "execute_comm_change" + component_name;
    message.adaptation_status = system_interfaces::msg::AdaptationStatus::STATUS_NOT_SET;

    relevantAdaptationTypes = {AdaptationType::ACTION_CHANGE_COMMUNICATION};

    std::string ServiceName = "/" + string_utils::strip(component_name, '/') + "/set_parameters";
    setServiceName(ServiceName);
}

/**
 * @brief Logs the experiment status for a given adaptation attempt.
 * @param adaptation_type The type of adaptation attempted.
 * @param adaptation_status The status of the adaptation.
 * @param success Indicates if the adaptation was successful.
 *
 * Gathers the current system time and logs adaptation attempts with their outcomes to the global experiment logger.
 */
void ExecuteCommChange::logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success)
{
    message.timestamp = experiment_logger->get_time();
    message.adaptation_type = adaptation_type;
    message.adaptation_status = adaptation_status;
    message.success = success;
    experiment_logger->silent_global(&message);
}

/**
 * @brief Sets up the request for communication changes based on input parameters.
 * @param request Shared pointer to the service request object to be populated.
 * @return Returns true upon successful setup of the request.
 *
 * Obtains the current adaptations from the blackboard, converts them to communication change adaptations, and populates the service request with the required parameters.
 * Then writes the list of adaptations without comm change adaptations back to the blackboard.
 * 
 */
bool ExecuteCommChange::setRequest(Request::SharedPtr &request)
{
    // Get the list of parametrization inputs from the input port
    auto InputList = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");
    std::vector<system_interfaces::msg::CommChangeInput> commInputList;
    auto splitAdaptations = adaptations_utils::splitAdaptations(InputList.value(), relevantAdaptationTypes);

    // store this as we need to know which adaptations are currently running in order to return adaptations that failed eventually
    current_adaptations = splitAdaptations.first;

    for (const auto& adaptation: current_adaptations)
    {
        commInputList.push_back(adaptations_utils::toCommChangeAdaptation(adaptation));
    }

    // set adaptations we do not use in this execution node back in the blackboard
    setOutput("adaptations", splitAdaptations.second);


    for (const auto& ParamInput : commInputList)
    {
        rcl_interfaces::msg::Parameter paramMsg;
        paramMsg.name = ParamInput.communication_name.data;
        paramMsg.value.string_value = ParamInput.value.data;
        paramMsg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        request->parameters.push_back(paramMsg);

        // logging
        logExperimentStatus(static_cast<int>(AdaptationType::ACTION_CHANGE_COMMUNICATION), static_cast<int>(AdaptationStatus::STATUS_ADAPTATION_TRIGGERED), true);
    }

    return true;
}

/**
* @brief Parses the response from parameter update requests.
* @param Result A vector of SetParametersResult messages indicating the success of each parameter update attempt.
* @return Returns true if all comm changes were successful, otherwise false.
*
*/
bool ExecuteCommChange::ParseParameterUpdateResponse(std::vector<rcl_interfaces::msg::SetParametersResult> &Result)
{
    std::string output = "";

    bool CommChangeUpdateSuccessful = true;
    
    auto bb_failed_adaptations = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations");
    std::vector<system_interfaces::msg::GenericAdaptation> failed_adaptations {};
    if (bb_failed_adaptations)
    {
        failed_adaptations = bb_failed_adaptations.value();
    }

    uint index = 0;
    for (auto &result : Result)
    {
        if (!result.successful)
        {
            output.append(result.reason);
            CommChangeUpdateSuccessful = false;
            failed_adaptations.push_back(current_adaptations[index]);
        } 
        index++;
    }

    current_adaptations.clear();

    setOutput("failed_adaptations", failed_adaptations);

    if (!CommChangeUpdateSuccessful)
    {
        RCLCPP_ERROR(logger(), "Failed to change comm type %s", output.c_str());
    }

    return CommChangeUpdateSuccessful;
}

/**
 * @brief Handles the receipt of a response from a ROS service call.
 * @param response Shared pointer to the Response message received.
 * @return Returns NodeStatus::SUCCESS if parsing is successful, otherwise NodeStatus::FAILURE.
 *
 * Upon receiving a response, this method evaluates the response's results and adjusts the node's status accordingly.
 */
NodeStatus ExecuteCommChange::onResponseReceived(const Response::SharedPtr &response)
{
    if (ParseParameterUpdateResponse(response->results))
    {
        // logging
        logExperimentStatus(static_cast<int>(AdaptationType::ACTION_CHANGE_COMMUNICATION), static_cast<int>(AdaptationStatus::STATUS_ADAPTATION_FINISHED), true);
        return NodeStatus::SUCCESS;
    }

    logExperimentStatus(static_cast<int>(AdaptationType::ACTION_CHANGE_COMMUNICATION), static_cast<int>(AdaptationStatus::STATUS_ADAPTATION_FINISHED), false);
    return NodeStatus::FAILURE;
}

/**
 * @brief Handles failure scenarios encountered during service operation.
 * @param error Error code associated with the service failure.
 * @return Returns NodeStatus::FAILURE in all cases.
 *
 * Logs errors encountered during adaptation attempts and returns a failure status for the node.
 */
NodeStatus ExecuteCommChange::onFailure(ServiceNodeErrorCode error)
{
    logExperimentStatus(static_cast<int>(AdaptationType::ACTION_CHANGE_COMMUNICATION), static_cast<int>(AdaptationStatus::STATUS_ADAPTATION_FINISHED), false);
    RCLCPP_ERROR(logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

/**
 * @brief Executes the tick function, polling for adaptations to apply.
 * @return Returns NodeStatus::SUCCESS if no adaptations are running or NodeStatus::RUNNING if further processing is needed.
 *
 * Always ticking the Base class tick if there is currently an adaptation running or if an adaptation should be executed.
 * Else, returns SUCCESS
 */
NodeStatus ExecuteCommChange::tick()
{
    // Get List of CommChangeInputs
    auto InputList = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");

    if (!InputList)
    {
        throw RuntimeError("Error getting input port [adaptations]", InputList.error());
    }

    auto splitAdaptations = adaptations_utils::splitAdaptations(InputList.value(), relevantAdaptationTypes);
    if (splitAdaptations.first.empty() && status() != NodeStatus::RUNNING)
    {
        return NodeStatus::SUCCESS;
    }

    return RosServiceNode<rcl_interfaces::srv::SetParameters>::tick();
}