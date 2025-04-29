#include "behaviortree_cpp/bt_factory.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "system_interfaces/msg/heartbeat.hpp"
#include "system_interfaces/msg/comm_change_input.hpp"
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"
#include <system_interfaces/msg/experiment_logging.hpp>
#include "bt_mape_k/util/logger.hpp"
using namespace BT;

// Define a graph using an adjacency list
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

class PlanningDecorator: public DecoratorNode
{
public:    
    PlanningDecorator(const std::string& name, const NodeConfiguration& config)
        : DecoratorNode(name, config) {
            auto bb_component_name = getInput<std::string>("component_name");
            if (!bb_component_name)
            {
                throw RuntimeError("Error getting input port [component_name]", bb_component_name.error());
            }
            component_name = bb_component_name.value();
            experiment_logger = BTLogger::get_global_logger();
            message.source = "planning_" + component_name;
        }

    static PortsList providedPorts()
    {
        return { 
            InputPort<std::string>("component_name"),
            InputPort<int>("hb_status"),
            InputPort<int>("clock"),
            InputPort<int>("lc_state"),
            BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations"),
            BidirectionalPort<bool>("need_redeploy"),
         };
    }
    NodeStatus tick() override;
    
private:
    void validateLifeCycleTransitions(std::vector<system_interfaces::msg::GenericAdaptation>& lcTransitions);

    // the timestamp when this node was last in a degraded or failure state
    int time_last_degraded = -1;
    std::string component_name;
    std::shared_ptr<BTLogger> experiment_logger;
    system_interfaces::msg::ExperimentLogging message;
};