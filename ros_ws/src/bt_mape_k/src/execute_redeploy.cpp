#include "bt_mape_k/execution.hpp"
#include "bt_mape_k/util/adaptation_utils.hpp"
using AdaptationStatus = system_interfaces::msg::AdaptationStatus;
using AdaptationType = system_interfaces::msg::AdaptationType;

/**
 * Logs the status of the experiment adaptation.
 * 
 * @param adaptation_type The type of adaptation being executed, represented as a uint8_t.
 * @param adaptation_status The status of the adaptation process, represented as a uint8_t.
 * @param success A boolean indicating whether the adaptation succeeded or failed.
 */
void ExecuteRedeploy::logExperimentStatus(uint8_t adaptation_type, uint8_t adaptation_status, bool success)
{
    message.timestamp = experiment_logger->get_time();
    message.source = "execute_redeploy" + component_name;
    message.adaptation_type = adaptation_type;
    message.adaptation_status = adaptation_status;
    message.success = success;
    experiment_logger->silent_global(&message);
}

/**
 * Constructor for ExecuteRedeploy, initializes the node and experiments logger.
 * 
 * @param name The name of the service node.
 * @param config The configuration parameters for the node.
 * @param params The ROS node parameters required for initialization.
 * 
 * @throws RuntimeError If there's an error getting the input port [component_name]
 * or if the node handle could not be locked.
 */
ExecuteRedeploy::ExecuteRedeploy(const std::string &name, const NodeConfiguration &config, const RosNodeParams& params)
    : RosServiceNode<lifecycle_msgs::srv::ChangeState>(name, config, params)
    {
        experiment_logger = BTLogger::get_global_logger();

        relevantAdaptationTypes = {AdaptationType::ACTION_REDEPLOY};

        auto component_res = getInput<std::string>("component_name");
        if (!component_res)
        {
            throw RuntimeError("Error getting input port [component_name]", component_res.error());
        }
        component_name = component_res.value();
        message.source += component_name;

        std::string ServiceName = component_name + "/change_state";
        setServiceName(ServiceName);

        // Extract the executable name
        size_t last_slash_pos = component_name.find_last_of("/");
        executable_name = component_name.substr(last_slash_pos + 1);

        // Extract the namespace name
        size_t first_slash_pos = component_name.find_first_of("/");
        size_t second_slash_pos = component_name.find_first_of("/", first_slash_pos + 1);
        package_name = component_name.substr(first_slash_pos + 1, second_slash_pos - first_slash_pos - 1);

        node_handle = params.nh.lock();
        if (!node_handle) // Check if the weak pointer was successfully locked
        {
            throw RuntimeError("[Monitoring] Could not lock the node handle, will not be able to provide the time");
        }
    }

/**
 * Main execution tick that performs the redeploy action.
 * Returns SUCCESS if there is currently nothing to do, executes the base class tick if a redeployment
 * has to be started or is currently happening.
 * 
 * @return NodeStatus indicating the result of the operation: SUCCESS, RUNNING or FAILURE.
 *
 * @throws RuntimeError If there's an error getting input port [adaptations].
 */
NodeStatus ExecuteRedeploy::tick()
{
    // Check if the node is still alive
    auto node_graph_interface = node_handle->get_node_graph_interface();
    std::vector<std::string> node_names = node_graph_interface->get_node_names();
    bool is_node_alive = false;
    for (const auto &name : node_names)
    {
        if (component_name == name)
        {
            is_node_alive = true;
        }
    }

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

    if (is_node_alive)
    {
        return RosServiceNode<lifecycle_msgs::srv::ChangeState>::tick();
    }
    else
    {
        experiment_logger->info("[ExecuteRedeploy] Process already dead");
    }

    // Restart the node
    if (restart_ros2_node(package_name, executable_name))
    {

        std::string command = "[ExecuteRedeploy] Successfully restarted node " + component_name + " from launch file with ros2 launch ";
        experiment_logger->info(command);
        //logExperimentStatus(AdaptationType::ACTION_REDEPLOY, AdaptationStatus::STATUS_ADAPTATION_FINISHED, true);
        return NodeStatus::SUCCESS;
    }

    //logExperimentStatus(AdaptationType::ACTION_REDEPLOY, AdaptationStatus::STATUS_ADAPTATION_FINISHED, false);
    std::cerr << "[ExecuteRedeploy] Failed to restart node: " << component_name << std::endl;
    return NodeStatus::FAILURE;
}

/**
 * Restarts a ROS2 node using the specified package and executable.
 * There has to be a launch file for this specific ROS node present that has the correct name.
 * E.g. if the ROS node name is /<namespace>/<node_name>, then there has to be a launch file called <node name>.launch.py
 * in the <namespace> ROS package
 * 
 * @param package The name of the package containing the launch file.
 * @param executable The name of the executable to be launched.
 * 
 * @return A boolean value indicating whether the restart operation was successful.
 */
bool ExecuteRedeploy::restart_ros2_node(const std::string &package, const std::string &executable)
{
    // restart with launch file instead of run command to ensure parameters stay the same
    std::string command = "ros2 launch " + package + " " + executable + ".launch.py &";
    int ret_code = system(command.c_str());
    return (ret_code == 0);
}

/**
 * Sets the request object with the appropriate transition for adaptation.
 * Updates the list of adaptations in the blackboard with the current list without the redeployment adaptation
 * 
 * @param request A shared pointer to the request being prepared for the transition.
 * 
 * @return A boolean value indicating whether the request was successfully set.
 */
bool ExecuteRedeploy::setRequest(Request::SharedPtr& request)
{
    auto InputList = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");
    std::vector<lifecycle_msgs::msg::Transition> transitionList;
    auto splitAdaptations = adaptations_utils::splitAdaptations(InputList.value(), relevantAdaptationTypes);
    setOutput("adaptations", splitAdaptations.second);
    logExperimentStatus(AdaptationType::ACTION_REDEPLOY, AdaptationStatus::STATUS_ADAPTATION_TRIGGERED, true);
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
    return true;
}

/**
 * Callback executed when a response is received from the service.
 * 
 * @param response A shared pointer to the response object received.
 * 
 * @return NodeStatus indicating the outcome of processing the response: SUCCESS or FAILURE.
 * 
 * Note: Attempts to restart the node after successfully receiving a response.
 */
NodeStatus ExecuteRedeploy::onResponseReceived(const Response::SharedPtr& response)
{
    if (!response->success)
    {
        logExperimentStatus(AdaptationType::ACTION_REDEPLOY, AdaptationStatus::STATUS_ADAPTATION_FINISHED, false);
        return NodeStatus::FAILURE;
    }

    if (restart_ros2_node(package_name, executable_name))
    {
        logExperimentStatus(AdaptationType::ACTION_REDEPLOY, AdaptationStatus::STATUS_ADAPTATION_FINISHED, true);
        return NodeStatus::SUCCESS;
    }
    else
    {
        logExperimentStatus(AdaptationType::ACTION_REDEPLOY, AdaptationStatus::STATUS_ADAPTATION_FINISHED, false);
        return NodeStatus::FAILURE;
    }
}

/**
 * Handles failure scenarios when the service node encounters errors.
 * 
 * @param error The error code associated with the node failure.
 * 
 * Note: Logs the failure of the adaptation process.
 */
NodeStatus ExecuteRedeploy::onFailure(ServiceNodeErrorCode error)
{
    logExperimentStatus(message.adaptation_type, AdaptationStatus::STATUS_ADAPTATION_FINISHED, false);
}