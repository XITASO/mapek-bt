#include "bt_mape_k/execution.hpp"
#include "bt_mape_k/util/adaptation_utils.hpp"
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
using AdaptationType = system_interfaces::msg::AdaptationType;

/**
 * @brief Constructor for ExecuteParametrization.
 * 
 * @param name A string representing the name of the node.
 * @param config A node configuration object containing specific node configurations.
 * @param params Parameters specific to the ROS service node behavior.
 * @throws RuntimeError If the input port "component_name" cannot be retrieved.
 *
 * Initializes the service name based on the component name and sets up experiment logging.
 */
ExecuteParametrization::ExecuteParametrization(const std::string& name, const NodeConfiguration& config, const RosNodeParams& params) : 
    RosServiceNode<rcl_interfaces::srv::SetParameters>(name, config, params)
{
    auto component_res = getInput<std::string>("component_name");
    if (!component_res)
    {
        throw RuntimeError("Error getting input port [component_name]", component_res.error());
    }
    std::string component_name = component_res.value();

    experiment_logger = BTLogger::get_global_logger();
    message = system_interfaces::msg::ExperimentLogging();
    message.source = "execute_parametrization" +  component_name;
    message.adaptation_status = system_interfaces::msg::AdaptationStatus::STATUS_NOT_SET;

    relevantAdaptationTypes = {AdaptationType::ACTION_DECREASE_PARAMETER, AdaptationType::ACTION_INCREASE_PARAMETER, AdaptationType::ACTION_SET_PARAMETER};

    std::string ServiceName = "/" + string_utils::strip(component_name, '/') + "/set_parameters";
    setServiceName(ServiceName);
}

/**
 * @brief Logs the status of the experiment adaptation.
 * 
 * @param adaptation_type The type of adaptation executed.
 * @param adaptation_status The status of the adaptation.
 * @param success Indicates whether the adaptation was successful.
 * 
 * Updates the experiment logging with the result of adaptations.
 */
void ExecuteParametrization::logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success)
{
    message.timestamp = experiment_logger->get_time();
    message.adaptation_type = adaptation_type;
    message.adaptation_status = adaptation_status;
    message.success = success;
    experiment_logger->silent_global(&message);
}

/**
 * @brief Sets the request for parameterization execution.
 * 
 * @param request Shared pointer to the request object for setting parameters.
 * @return True if the request is set successfully, otherwise false.
 * 
 * Retrieves the adaptation inputs and prepares the request object with appropriate parameters.
 * Removes the parametrization adaptations from the adaptations entry in the blackboard.
 */
bool ExecuteParametrization::setRequest(Request::SharedPtr &request)
{
    RCLCPP_INFO(logger(), "Set request in parametrization execution");
    // Get the list of parametrization inputs from the input port
    auto InputList = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");
    if (!InputList)
    {
        RCLCPP_ERROR(logger(), "Failed to get parametrization_input: %s", InputList.error().c_str());
        return false;
    }

    std::vector<system_interfaces::msg::ParametrizationInput> paramInputList;
    auto splitAdaptations = adaptations_utils::splitAdaptations(InputList.value(), relevantAdaptationTypes);

    // store this as we need to know which adaptations are currently running in order to return adaptations that failed eventually
    current_adaptations = splitAdaptations.first;

    for (const auto& adaptation: current_adaptations)
    {
        paramInputList.push_back(adaptations_utils::toParametrizationAdaptation(adaptation));
    }

    setOutput("adaptations", splitAdaptations.second);

    for (const auto& ParamInput : paramInputList)
    {
        request->parameters.push_back(ParamInput.parameter);
    }
    logExperimentStatus(static_cast<int>(AdaptationType::ACTION_SET_PARAMETER), static_cast<int>(AdaptationStatus::STATUS_ADAPTATION_TRIGGERED), true);
    return true;
}

/**
 * @brief Parses the response of the parameter update service.
 * 
 * @param Result A vector holding the results of parameter updates.
 * @return True if the parameter updates were successful, otherwise false.
 *
 * Analyzes the response received from the service to verify successful parameter updates.
 * Logs errors if any adaptations fail and updates the list of failed adaptations.
 */
bool ExecuteParametrization::ParseParameterUpdateResponse(std::vector<rcl_interfaces::msg::SetParametersResult> &Result)
{
    std::string output = "";

    bool ParameterUpdateSuccessful = true;

    // no need to verify if there; we already did this in tick()
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
            ParameterUpdateSuccessful = false;
            failed_adaptations.push_back(current_adaptations[index]);
        } 
        index++;
    }

    current_adaptations.clear();
    setOutput("failed_adaptations", failed_adaptations);

    if (!ParameterUpdateSuccessful)
    {
        RCLCPP_ERROR(logger(), "Failed to set parameters %s", output.c_str());
    }

    return ParameterUpdateSuccessful;
}

/**
 * @brief Callback for when a response is received from the parameter setting service.
 * 
 * @param response Shared pointer to the response received.
 * @return NodeStatus indicating the success or failure of the operation.
 * 
 * Processes results of a parameter update response and logs the final adaptation status.
 */
NodeStatus ExecuteParametrization::onResponseReceived(const Response::SharedPtr &response)
{
    if (ParseParameterUpdateResponse(response->results))
    {
        logExperimentStatus(static_cast<int>(AdaptationType::ACTION_SET_PARAMETER), static_cast<int>(AdaptationStatus::STATUS_ADAPTATION_FINISHED), true);
        return NodeStatus::SUCCESS;
    }

    logExperimentStatus(static_cast<int>(AdaptationType::ACTION_SET_PARAMETER), static_cast<int>(AdaptationStatus::STATUS_ADAPTATION_FINISHED), false);
    return NodeStatus::FAILURE;
}

/**
 * @brief Callback executed on service node failure.
 * 
 * @param error Error code indicating the type of failure.
 * @return NodeStatus indicating the failure state.
 * 
 * Logs adaptation status as failed and outputs an error message upon encountering failures during node operations.
 */
NodeStatus ExecuteParametrization::onFailure(ServiceNodeErrorCode error)
{
    logExperimentStatus(static_cast<int>(AdaptationType::ACTION_SET_PARAMETER), static_cast<int>(AdaptationStatus::STATUS_ADAPTATION_FINISHED), false);
    RCLCPP_ERROR(logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

/**
 * @brief Ticks the behavior tree node for executing parameterization adaptations.
 * 
 * @return NodeStatus indicating whether the node successfully performed its task.
 * 
 * Always ticking the Base class tick if there is currently an adaptation running or if an adaptation should be executed.
 * Else, returns SUCCESS
 */
NodeStatus ExecuteParametrization::tick()
{
    // Get List of ParameterizationInputs
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