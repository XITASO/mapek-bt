#include "bt_mape_k/analysing.hpp"
#include "bt_mape_k/analysis_decorator/RuleParser.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"

using AdaptationType= system_interfaces::msg::AdaptationType;

AnalysisDecorator::AnalysisDecorator(const std::string& name, const NodeConfiguration& config, std::string rules_path)
    : DecoratorNode(name, config)   
    {
        experiment_logger = BTLogger::get_global_logger();
        auto component_name = getInput<std::string>("component_name");
        message.source = "analysis_" + component_name.value();
        RuleParser p(config.blackboard);
        rules = p.parse(rules_path);
    }

NodeStatus AnalysisDecorator::tick() {
    // check and get adaptations and component
    auto bb_component = getInput<std::string>("component_name");
    auto bb_max_failures = getInput<uint>("max_failures");
    auto bb_existing_adaptations = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations");
    auto bb_failed_adaptations = getInput<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations");
    std::string component {};
    uint max_failures {};
    std::vector<system_interfaces::msg::GenericAdaptation> adaptations {};
    std::vector<system_interfaces::msg::GenericAdaptation> failed_adaptations {};

    if (!bb_existing_adaptations){
        adaptations = std::vector<system_interfaces::msg::GenericAdaptation>();
    }else{
        adaptations = bb_existing_adaptations.value();
    }
    if (!bb_failed_adaptations){
        failed_adaptations = std::vector<system_interfaces::msg::GenericAdaptation>();
    }else{
        failed_adaptations = bb_failed_adaptations.value();
    }
    if (!bb_component){
        throw std::runtime_error("Error getting input port [component_name]");
    }
    component = bb_component.value();
    if (!bb_max_failures){
        max_failures = 5;
    }else{
        max_failures = bb_max_failures.value();
    }
    //std::cout<<"starting analysis for "<<component<<std::endl;
    // get adaptation for all rules for specific component
    uint adaptation_ctr = 0;
    for (const auto & rule: rules){
        // check triggers
        auto maybe_adaptation = rule->execute(component); 
        
        if (maybe_adaptation){
            message.timestamp = experiment_logger->get_time();
            message.rule_name = rule->getName();
            message.adaptation_type = maybe_adaptation.value().action;
            //experiment_logger->silent_global(&message);
            adaptations.push_back(maybe_adaptation.value());
            //std::cout<<component<<" got adaptation of type "<<(int)maybe_adaptation.value().action<<std::endl;
            adaptation_ctr += 1;
        }
    }

    // get adaptations to retry
    auto retry_adaptations = evaluateFailedAdaptations(failed_adaptations, max_failures);
    
    // merge them 
    adaptations = spiceIncomingAdaptationsWithFailedOnes(adaptations, retry_adaptations);
    // remove garbage
    filterInsaneAdaptations(adaptations);

    // call even if we did not find any adaptation, to reset them 
    //sortAndWriteAdaptations(adaptations);
    

    setOutput<std::vector<system_interfaces::msg::GenericAdaptation>>("adaptations", adaptations);
    setOutput<std::vector<system_interfaces::msg::GenericAdaptation>>("failed_adaptations",{});

    // tick the child
    auto child_status = child_node_->executeTick();
    return BT::NodeStatus::SUCCESS;
}

std::vector<system_interfaces::msg::GenericAdaptation> AnalysisDecorator::evaluateFailedAdaptations(std::vector<system_interfaces::msg::GenericAdaptation> new_failed_adaptations, uint max_failures){
    //std::cout<<"incoming failed: "<<new_failed_adaptations.size()<<std::endl; 
    // first add all failed adaptations to the cache, respectively increase the counter
    for(auto failed_adaptation : new_failed_adaptations){
        if (!failed_adaptation_cache.count(failed_adaptation)){
            failed_adaptation_cache[failed_adaptation] = 0;
        }else{
            failed_adaptation_cache[failed_adaptation] += 1;
        }
    }

    // second, all adaptations in the cache, but not in the list of failed adaptations must therefore be successful adaptations on a later try --> delete them from cache
    for (auto it = failed_adaptation_cache.cbegin(); it != failed_adaptation_cache.cend(); ){
        // not found
        if (std::find(new_failed_adaptations.begin(), new_failed_adaptations.end(), it->first) == new_failed_adaptations.end()){
            it = failed_adaptation_cache.erase(it);
        }else{
            ++it;
            
        }
    }
    // whats the point of redeploying multiple times? --> don't
    bool got_extra_redeploy = false;

    std::vector<system_interfaces::msg::GenericAdaptation> retry_adaptations;
    // all adaptations still left in cache must be tried again, or lead to redeployment
    for (auto it = failed_adaptation_cache.cbegin(); it != failed_adaptation_cache.cend(); ){
        if (it->second == max_failures){
            if (!got_extra_redeploy){
                got_extra_redeploy = true;
                system_interfaces::msg::GenericAdaptation redeploy_adaptation;
                redeploy_adaptation.action = system_interfaces::msg::AdaptationType::ACTION_REDEPLOY;
                retry_adaptations.push_back(redeploy_adaptation);
                experiment_logger->warn("ordered redeployment");
            }
            experiment_logger->warn("removed expired adaptation");
            it = failed_adaptation_cache.erase(it);
        }else{
            retry_adaptations.push_back(it->first);
            ++it;
            
        }
    }

    // note, in here may be duplicated adaptations, those must be handeled later 
    return retry_adaptations;
}

std::vector<system_interfaces::msg::GenericAdaptation> AnalysisDecorator::spiceIncomingAdaptationsWithFailedOnes(std::vector<system_interfaces::msg::GenericAdaptation> new_adaptations, std::vector<system_interfaces::msg::GenericAdaptation> retry_adaptations){
    auto combined_adaptations = new_adaptations;
    combined_adaptations.insert(combined_adaptations.end(), retry_adaptations.begin(), retry_adaptations.end());
    // Obacht! -> There may be duplicated adaptations --> delete those
    auto it = unique(combined_adaptations.begin(), combined_adaptations.end());
    combined_adaptations.erase(it, combined_adaptations.end()); 

    return combined_adaptations;
}

void AnalysisDecorator::filterInsaneAdaptations(std::vector<system_interfaces::msg::GenericAdaptation>& adaptations){
    // remove all insane adaptations
    for(auto it = adaptations.begin(); it != adaptations.end(); ){   
        if(!adaptations_utils::sanityCheck(*it)){
            std::cout<<"removed insane adaptation\n";
            it = adaptations.erase(it);
        }else
            it++;
    }
}

void AnalysisDecorator::sortAndWriteAdaptations(std::vector<system_interfaces::msg::GenericAdaptation> adaptations){
    std::vector<system_interfaces::msg::ParametrizationInput> parametrization_adaptations {};
    std::vector<system_interfaces::msg::CommChangeInput> communication_adaptations {};
    std::vector<lifecycle_msgs::msg::Transition> lifecycle_adaptations {};
    bool need_redeploy_adaptation {};
    //std::cout<<"total #adaptations: "<<adaptations.size()<<std::endl;


    for (const auto& adaptation : adaptations){
        switch(adaptation.action){
            case AdaptationType::ACTION_CHANGE_COMMUNICATION:

                communication_adaptations.push_back(adaptations_utils::toCommChangeAdaptation(adaptation));
                break;
            // intentional fall through
            case AdaptationType::ACTION_SET_PARAMETER:
            case AdaptationType::ACTION_INCREASE_PARAMETER:
            case AdaptationType::ACTION_DECREASE_PARAMETER:
                parametrization_adaptations.push_back(adaptations_utils::toParametrizationAdaptation(adaptation));
                break;
            // intentional fall through
            case AdaptationType::ACTION_ACTIVATE:
            case AdaptationType::ACTION_DEACTIVATE:
            case AdaptationType::ACTION_RESTART:
            {
                auto lc_transitions = adaptations_utils::toLifecyleAdaptation(adaptation);
                lifecycle_adaptations.insert(lifecycle_adaptations.end(), lc_transitions.begin(), lc_transitions.end());
                break;
            }
            case AdaptationType::ACTION_REDEPLOY:
                need_redeploy_adaptation  = true;
                break;
            default:
                std::cout<<"Unknown or unset adaptation type\n";
        }
    }

    setOutput<std::vector<system_interfaces::msg::ParametrizationInput>>("parametrization_input", parametrization_adaptations);
    setOutput<std::vector<system_interfaces::msg::CommChangeInput>>("communication_input", communication_adaptations);
    setOutput<std::vector<lifecycle_msgs::msg::Transition>>("lifecycle_input", lifecycle_adaptations);
    setOutput<bool>("need_redeploy", need_redeploy_adaptation);
}