#pragma once

#include <vector>
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "behaviortree_cpp/blackboard.h"
#include <optional>
#include "bt_mape_k/util/string_utils.hpp"

#define Adaptation std::function<system_interfaces::msg::GenericAdaptation()>

// Helper factory to generate Adaptations (lambda functions)
struct AdaptationFactory{
    // creates Adaptations
    Adaptation produce(std::vector<std::string>, BT::Blackboard::Ptr);
    
    private:
    // creates Adaptations w/o parameters, input: action type
    Adaptation produceNoParamAdaptation(uint8_t type);

    // creates Adaptations w/ parameters, intput: action type and params as string to parse
    Adaptation produceParametrizedAdaptation(uint8_t type, std::vector<std::string>, BT::Blackboard::Ptr);
};