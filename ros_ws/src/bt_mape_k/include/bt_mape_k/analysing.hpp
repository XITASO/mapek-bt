#include "behaviortree_cpp/bt_factory.h"

#include "system_interfaces/msg/generic_adaptation.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "system_interfaces/msg/comm_change_input.hpp"
#include "system_interfaces/msg/parametrization_input.hpp"
#include "bt_mape_k/analysis_decorator/Rule.hpp"
#include "bt_mape_k/util/adaptation_utils.hpp"
#include "bt_mape_k/util/logger.hpp"
#include <unordered_map>




using namespace BT;

class AnalysisDecorator : public DecoratorNode
{
public:
    AnalysisDecorator(const std::string& name, const NodeConfiguration& config, std::string rules_path);

    static PortsList providedPorts()
    {
        return {
            InputPort<uint>("fails_2_redeploy", 5, "how often can a single adaptation fail, until drastic measures are taken?"),
            InputPort<std::string>("component_name"),
            BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations"),
            BidirectionalPort<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations"),
        };
    }

    NodeStatus tick() override;

    private: 
        void sortAndWriteAdaptations(std::vector<system_interfaces::msg::GenericAdaptation>);
        std::vector<system_interfaces::msg::GenericAdaptation> evaluateFailedAdaptations(std::vector<system_interfaces::msg::GenericAdaptation> failed_adaptations, uint max_failures);
        std::vector<system_interfaces::msg::GenericAdaptation> spiceIncomingAdaptationsWithFailedOnes(std::vector<system_interfaces::msg::GenericAdaptation> new_adaptations, std::vector<system_interfaces::msg::GenericAdaptation> retry_adaptations);
        void filterInsaneAdaptations(std::vector<system_interfaces::msg::GenericAdaptation>& adaptations);

        std::vector<std::shared_ptr<Rule>> rules;
        
        // Note, to use this map the hash fuction template specialization given in adaptation_utils is needed
        std::unordered_map<system_interfaces::msg::GenericAdaptation, uint> failed_adaptation_cache {};

        std::shared_ptr<BTLogger> experiment_logger;
        system_interfaces::msg::ExperimentLogging message;
};