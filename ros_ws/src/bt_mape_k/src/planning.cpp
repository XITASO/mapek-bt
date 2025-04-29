#include "bt_mape_k/planning.hpp"
#include "bt_mape_k/util/adaptation_utils.hpp"
using AdaptationType = system_interfaces::msg::AdaptationType;

/**
 * @brief Executes a behavior tree node for planning, which evaluates the necessary adaptations for a system
 *        based on its current state. The node assesses various inputs such as adaptation actions, heartbeat status,
 *        dependency graph, and the current time to determine the system's adaptability and respond accordingly.
 *        It updates the dependency graph and other execution parameters as needed.
 *
 * @return The status of the child node after execution (NodeStatus).
 */
NodeStatus PlanningDecorator::tick()
{
    auto bb_adaptations = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");
    if (!bb_adaptations)
    {
        throw RuntimeError("Error getting input port [adaptations]", bb_adaptations.error());
    }
    std::vector<system_interfaces::msg::GenericAdaptation> adaptations = bb_adaptations.value();

    size_t redeployCount = adaptations_utils::countAdaptations(adaptations,std::vector<uint8_t>{AdaptationType::ACTION_REDEPLOY});
    auto splitAdaptations = adaptations_utils::splitAdaptations(adaptations, std::vector<uint8_t>{AdaptationType::ACTION_ACTIVATE, AdaptationType::ACTION_DEACTIVATE, AdaptationType::ACTION_RESTART});
    auto lcTransitions = splitAdaptations.first;
    adaptations = splitAdaptations.second;
    // check if there are any lifecycle transitions that are not feasible because we are already in the right state
    validateLifeCycleTransitions(lcTransitions);

    // if we still need to transition, then we merge the lifecycle transitions back into the current adaptations
    if (!lcTransitions.empty())
    {
        adaptations.insert(adaptations.end(), lcTransitions.begin(), lcTransitions.end());
    }

    // log all adaptations which are actually being used right here
    for (const auto &adaptation : adaptations) {
        message.timestamp = experiment_logger->get_time();
        message.rule_name = adaptation.parameter.name;
        message.adaptation_type = adaptation.action;
        experiment_logger->silent_global(&message);
    }

    setOutput("adaptations", adaptations);

    if (redeployCount > 0)
    {
        experiment_logger->info("Redeploy needed for " + component_name + " with adaptation count = " + std::to_string(adaptations.size()));
    }

    auto child_status = child_node_->executeTick();
    return child_status;
}

/**
 * @brief Validates lifecycle transition adaptations to ensure they are needed for the current lifecycle state.
 *        Removes unnecessary transitions that would attempt to change to the same lifecycle state.
 *
 * @param[in,out] lcTransitions Vector of lifecycle transition adaptations to be validated.
 */
void PlanningDecorator::validateLifeCycleTransitions(std::vector<system_interfaces::msg::GenericAdaptation>& lcTransitions)
{
    if (lcTransitions.empty())
        return;
    auto bb_lc_state = getInput<int>("lc_state");
    if (!bb_lc_state)
    {
        throw RuntimeError("Error getting input port [lc_state]", bb_lc_state.error());
    }
    int current_lc_state = bb_lc_state.value();
    if (lcTransitions[0].action == system_interfaces::msg::AdaptationType::ACTION_ACTIVATE && current_lc_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        lcTransitions.erase(lcTransitions.begin());
    }
    else if (lcTransitions[0].action == system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE && current_lc_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        lcTransitions.erase(lcTransitions.begin());
    }
}
