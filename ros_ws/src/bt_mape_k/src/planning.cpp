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
    auto bb_dep_graph = getInput<std::pair<Graph,std::unordered_map<std::string, std::pair<Vertex, bool>>>>("dependency_graph");
    if (!bb_dep_graph)
    {
        throw RuntimeError("Error getting input port [dependency_graph]", bb_dep_graph.error());
    }
    auto graph = bb_dep_graph.value().first;
    auto vertex_map = bb_dep_graph.value().second;

    auto bb_adaptations = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");
    if (!bb_adaptations)
    {
        throw RuntimeError("Error getting input port [adaptations]", bb_adaptations.error());
    }
    std::vector<system_interfaces::msg::GenericAdaptation> adaptations = bb_adaptations.value();

    auto bb_hb_status = getInput<int>("hb_status");
    if (!bb_hb_status)
    {
        throw RuntimeError("Error getting input port [hb_status]", bb_hb_status.error());
    }
    int hb_status = bb_hb_status.value();

    auto bb_clock = getInput<int>("clock");
    if (!bb_clock)
    {
        throw RuntimeError("Error getting input port [clock]", bb_clock.error());
    }
    int current_time = bb_clock.value();

    bool need_redeploy = false;
    
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
        need_redeploy = true;
        experiment_logger->info("Redeploy needed for " + component_name + " with adaptation count = " + std::to_string(adaptations.size()));
    }

    if (hb_status == static_cast<int>(system_interfaces::msg::Heartbeat::HB_STATUS_OK) ||
        hb_status == static_cast<int>(system_interfaces::msg::Heartbeat::HB_STATUS_FAILURE))
    {
        time_last_degraded = current_time;
    }

    // time in milliseconds, will be zero if we are currently degraded
    int time_since_last_degradation = current_time - time_last_degraded;

    if (hb_status == static_cast<int>(system_interfaces::msg::Heartbeat::HB_STATUS_OK)
        || time_since_last_degradation > 1e3 // the time since we last were degraded is 1 second away
        || time_last_degraded == -1) // default value, this node was until now never degraded
    {
        // set ourselves to not busy anymore if also the current time is more away from the last time we were degraded
       vertex_map[component_name].second = false;
    }

    // If nodes we are dependent on, degraded, we will not do any lifecycle changes or redeploys or we are currently busy ourselves
    if(areDependentNodesDegraded(vertex_map, graph, component_name) || vertex_map[component_name].second)
    {
        auto withoutLCChanges = adaptations_utils::splitAdaptations(adaptations, {AdaptationType::ACTION_ACTIVATE, AdaptationType::ACTION_DEACTIVATE, AdaptationType::ACTION_RESTART});
        auto withoutCommChange = adaptations_utils::splitAdaptations(withoutLCChanges.second, {AdaptationType::ACTION_CHANGE_COMMUNICATION});
        auto withoutRedeploy = adaptations_utils::splitAdaptations(withoutCommChange.second, {AdaptationType::ACTION_REDEPLOY});
        experiment_logger->info("Setting only parametrization adaptations for " + component_name);
        setOutput("adaptations", withoutRedeploy.second);
    }
    
    // If we will be busy or are degraded, we will tell this to the other nodes
    if (
        need_redeploy == true || 
        hb_status == static_cast<int>(system_interfaces::msg::Heartbeat::HB_STATUS_DEGRADED) || 
        hb_status == static_cast<int>(system_interfaces::msg::Heartbeat::HB_STATUS_FAILURE))
    {
        vertex_map[component_name].second = true;
    }

    setOutput("dependency_graph", std::make_pair(graph, vertex_map));

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


/**
 * @brief Identifies if any nodes that the current node is dependent on are degraded. This decision influences
 *        whether adaptation actions should be restricted to parametrization changes.
 *
 * @param vertex_map A map of vertex names to their corresponding Vertex objects and degradation statuses.
 * @param g The graph representing the dependency relationships between nodes.
 * @param vertex_name The name of the vertex (current node) to check dependencies for.
 * @return True if any dependent nodes are degraded, false otherwise.
 */ 
bool PlanningDecorator::areDependentNodesDegraded(std::unordered_map<std::string, std::pair<Vertex, bool>>& vertex_map, Graph g, std::string vertex_name)
{
    Vertex startVertex = vertex_map.at(vertex_name).first;

    // Check if the node itself is degraded
    if (vertex_map.at(vertex_name).second) {
        return true;
    }

    std::set<Vertex> visited;
    std::deque<Vertex> queue; 
    queue.push_back(startVertex);

    while (!queue.empty()) {
        Vertex currentVertex = queue.front();
        queue.pop_front();

        if (visited.find(currentVertex) != visited.end()) {
            continue;
        }
        visited.insert(currentVertex);

        // Iterate over outgoing edges - checking forward dependencies
        Graph::out_edge_iterator out_i, out_end;
        boost::tie(out_i, out_end) = boost::out_edges(currentVertex, g);
        for (; out_i != out_end; ++out_i) {
            Vertex targetVertex = boost::target(*out_i, g);

            // Check if the vertex targets a degraded node
            for (const auto& kv : vertex_map) {
                if (kv.second.first == targetVertex && kv.second.second) {
                    return true;
                }
            }

            queue.push_back(targetVertex);
        }
    }
    return false;
}