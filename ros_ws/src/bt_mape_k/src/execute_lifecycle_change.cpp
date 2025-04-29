#include "bt_mape_k/execution.hpp"
#include "bt_mape_k/util/adaptation_utils.hpp"
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
using AdaptationType = system_interfaces::msg::AdaptationType;

/**
 * @brief Constructor for the ExecuteLifeCycle class.
 *
 * Initialize the service node and set up parameters necessary for lifecycle management.
 * Retrieves component name, prepares the logging message, configures adaptation types,
 * and sets the ROS service name for lifecycle changes.
 *
 * @param name The name of the behavior tree node.
 * @param config Configuration details for the behavior tree node.
 * @param params Parameters needed for ROS node execution.
 *
 * @throws RuntimeError if the component name cannot be retrieved from input ports.
 */
ExecuteLifeCycle::ExecuteLifeCycle(const std::string& name, const NodeConfiguration& config, const RosNodeParams& params)
: RosServiceNode<system_interfaces::srv::SetLifecycleChanges>(name, config, params)
{
    // Initialize the random publisher
    experiment_logger = BTLogger::get_global_logger();

    auto component_res = getInput<std::string>("component_name");
    if (!component_res)
    {
        throw RuntimeError("Error getting input port [component_name]", component_res.error());
    }
    component_name = component_res.value();

    message = system_interfaces::msg::ExperimentLogging();
    message.source = "execute_lifecycle" + component_name;
    message.adaptation_status = system_interfaces::msg::AdaptationStatus::STATUS_NOT_SET;

    relevantAdaptationTypes = {AdaptationType::ACTION_ACTIVATE, AdaptationType::ACTION_DEACTIVATE, AdaptationType::ACTION_RESTART};

    std::string ServiceName = "/" + string_utils::strip(component_name, '/') + "/set_lifecycle_changes";
    setServiceName(ServiceName);
}

/**
 * @brief Logs the current status of an adaptation experiment.
 *
 * Records the timestamp, type, and status of adaptation along with a success flag.
 * Messages are logged using the global experiment logger.
 *
 * @param adaptation_type Type of adaptation being executed.
 * @param adaptation_status Current status of the adaptation process.
 * @param success Boolean indicating whether the adaptation was successful.
 */
void ExecuteLifeCycle::logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success)
{
    message.timestamp = experiment_logger->get_time();
    message.adaptation_type = adaptation_type;
    message.adaptation_status = adaptation_status;
    message.success = success;
    experiment_logger->silent_global(&message);
}

/**
 * @brief Sets the request for lifecycle transition adaptations.
 *
 * Configures the service request based on input adaptations and current lifecycle states.
 * Splits adaptations into currently applicable and non-applicable types,
 * and determines the type of lifecycle change occurring (e.g., activate, deactivate, restart).
 * Logs the initiation of the lifecycle execution request.
 *
 * @param request Shared pointer to the service request to be configured.
 * @return true if the request was set successfully, false otherwise.
 *
 * @throws RuntimeError if input adaptations cannot be retrieved from input ports.
 */
bool ExecuteLifeCycle::setRequest(Request::SharedPtr &request)
{
    // RCLCPP_INFO(logger(), "Set request in lifecycle execution");
    experiment_logger->info("Set request in lifecycle execution for " + component_name);
    // Get the list of parametrization inputs from the input port
    auto InputList = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");
    std::vector<lifecycle_msgs::msg::Transition> transitionList;
    auto splitAdaptations = adaptations_utils::splitAdaptations(InputList.value(), relevantAdaptationTypes);

    // store this as we need to know which adaptations are currently running in order to return adaptations that failed eventually
    current_adaptations = splitAdaptations.first;
    for (const auto& transition: current_adaptations)
    {
        for (const auto& lcAdaptation: adaptations_utils::toLifecyleAdaptation(transition))
        {
            transitionList.push_back(lcAdaptation);
        }
    }

    setOutput("adaptations", splitAdaptations.second);

    request->transitions = transitionList;

    // As we only have one case in which multiple lifecycle change can happen, this is the restart
    // Will be different, if we say that we include configuring of a node in the AdaptationTypes
    int adaptation_type;
    if (request->transitions.size() > 1)
    {
        adaptation_type = static_cast<int>(system_interfaces::msg::AdaptationType::ACTION_RESTART);
    }
    else
    {
        if (request->transitions[0].id == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)
        {
            adaptation_type = static_cast<int>(system_interfaces::msg::AdaptationType::ACTION_ACTIVATE);
        }
        else if (request->transitions[0].id == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)
        {
            adaptation_type = static_cast<int>(system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE);
        }
    }
    logExperimentStatus(adaptation_type, AdaptationStatus::STATUS_ADAPTATION_TRIGGERED, false);

    return true;
}

/**
 * @brief Response handler for lifecycle change requests.
 *
 * Processes the results of lifecycle change requests and determines the success or failure of each transition.
 * Updates the lifecycle state of the ROS component that was changed and failed adaptations output ports accordingly.
 * Logs the results and updates the experiment status.
 *
 * @param response Shared pointer to the service response containing results of lifecycle transitions.
 * @return NodeStatus::SUCCESS if all transitions are successful, NodeStatus::FAILURE otherwise.
 */
NodeStatus ExecuteLifeCycle::onResponseReceived(const Response::SharedPtr &response)
{
    std::string log_string = "Response received in lifecycle execution " + component_name;
    RCLCPP_INFO(logger(), log_string.c_str());
    bool LifecycleChangeSuccessful = true;
    
    auto bb_failed_adaptations = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations");
    std::vector<system_interfaces::msg::GenericAdaptation> failed_adaptations {};
    if (bb_failed_adaptations)
    {
        failed_adaptations = bb_failed_adaptations.value();
    }
    
    uint index = 0;
    for (const auto &result : response->results)
    {
        if (!result)
        {
            LifecycleChangeSuccessful = false;
            failed_adaptations.push_back(current_adaptations[index]);
        } 
        index++;
    }

    if (LifecycleChangeSuccessful)
    {
        int action = current_adaptations[current_adaptations.size() - 1].action;
        if (action == static_cast<int>(system_interfaces::msg::AdaptationType::ACTION_ACTIVATE))
        {
            setOutput("lc_state", static_cast<int>(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
        }
        else if( action == static_cast<int>(system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE))
        {
            setOutput("lc_state", static_cast<int>(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE));
        }
    }

    current_adaptations.clear();

    setOutput("failed_adaptations", failed_adaptations);

    if (!LifecycleChangeSuccessful)
    {
        experiment_logger->warn("Failed to change Lifecycle." + component_name);
        logExperimentStatus(message.adaptation_type, AdaptationStatus::STATUS_ADAPTATION_FINISHED, false);
        return NodeStatus::FAILURE;
    }

    

    logExperimentStatus(message.adaptation_type, AdaptationStatus::STATUS_ADAPTATION_FINISHED, true);
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Failure handler for lifecycle service node.
 *
 * Records the failure status of an adaptation process due to an error in service node execution.
 * Logs the error and adjusts the experiment status accordingly.
 *
 * @param error Enumeration code representing the type of error encountered.
 * @return NodeStatus::FAILURE always returns failure status upon error.
 */
NodeStatus ExecuteLifeCycle::onFailure(ServiceNodeErrorCode error)
{
    logExperimentStatus(message.adaptation_type, AdaptationStatus::STATUS_ADAPTATION_FINISHED, false);
    RCLCPP_ERROR(logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

/**
 * @brief Tick function for executing lifecycle changes in behavior tree.
 *
 * Retrieves and processes input adaptations for lifecycle transitions. Determines if any lifecycle transitions
 * are applicable, and executes any necessary changes through service node mechanisms.
 * The tick of the base class will be only executed if there currently is a lifecycle transition to be started
 * or there is currently one running
 *
 * @return NodeStatus::SUCCESS if no applicable adaptations are present, otherwise inherits the status from
 *         RosServiceNode tick execution.
 *
 * @throws RuntimeError if input adaptations cannot be retrieved.
 */
NodeStatus ExecuteLifeCycle::tick()
{
    // Get List of LifeCycleInputs
    auto InputList = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");

    // No List able to be retrieved
    if (!InputList)
    {
        throw RuntimeError("Error getting input port [adaptations]", InputList.error());
    }

    auto splitAdaptations = adaptations_utils::splitAdaptations(InputList.value(), relevantAdaptationTypes);

    if (splitAdaptations.first.empty() && status() != NodeStatus::RUNNING)
    {
        return NodeStatus::SUCCESS;
    }

    return RosServiceNode<system_interfaces::srv::SetLifecycleChanges>::tick();
}