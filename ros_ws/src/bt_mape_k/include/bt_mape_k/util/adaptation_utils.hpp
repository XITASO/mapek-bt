#pragma once

#include <string>
#include "lifecycle_msgs/msg/transition.hpp"
#include "system_interfaces/msg/comm_change_input.hpp"
#include "system_interfaces/msg/parametrization_input.hpp"
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "system_interfaces/msg/generic_adaptation.hpp"

namespace adaptations_utils{

uint8_t deduceAdaptationType(std::string type);

system_interfaces::msg::CommChangeInput toCommChangeAdaptation(system_interfaces::msg::GenericAdaptation);
system_interfaces::msg::ParametrizationInput toParametrizationAdaptation(system_interfaces::msg::GenericAdaptation);
std::vector<lifecycle_msgs::msg::Transition> toLifecyleAdaptation(system_interfaces::msg::GenericAdaptation);

system_interfaces::msg::GenericAdaptation toGenericAdaptation(system_interfaces::msg::CommChangeInput);
system_interfaces::msg::GenericAdaptation toGenericAdaptation(system_interfaces::msg::ParametrizationInput);
system_interfaces::msg::GenericAdaptation toGenericAdaptation(lifecycle_msgs::msg::Transition);


bool sanityCheck(system_interfaces::msg::GenericAdaptation);
std::pair<std::vector<system_interfaces::msg::GenericAdaptation>, std::vector<system_interfaces::msg::GenericAdaptation>>
  splitAdaptations(const std::vector<system_interfaces::msg::GenericAdaptation>& adaptations,
                 const std::vector<uint8_t>& desired_actions);

size_t countAdaptations(const std::vector<system_interfaces::msg::GenericAdaptation>& adaptations, const std::vector<uint8_t>& desired_actions);

}

// hash specialization, to use GenericAdaptation in unordered_map etc
template <>
struct std::hash<system_interfaces::msg::GenericAdaptation>
{
  std::size_t operator()(const system_interfaces::msg::GenericAdaptation& adaptation) const
  {
    using std::size_t;
    using std::hash;
    using std::string;

    std::hash<std::string> string_hash;
    std::hash<uint8_t> uint8_hash;
    std::hash<double> double_hash;
    std::hash<bool> bool_hash;

    // Combine the hash values of the fields
    // The magic numbers here are really magic numbers in order to get a good hash value
    // seems to be the way2go
    size_t result = uint8_hash(adaptation.action);

    result ^= string_hash(adaptation.parameter.name) + 0x9e3779b9 + (result << 6) + (result >> 2);
    result ^= uint8_hash(adaptation.parameter.value.type) + 0x9e3779b9 + (result << 6) + (result >> 2);

    result ^= uint8_hash(adaptation.parameter.value.type) + 0x9e3779b9 + (result << 6) + (result >> 2);
    result ^= string_hash(adaptation.parameter.value.string_value) + 0x9e3779b9 + (result << 6) + (result >> 2);
    result ^= double_hash(adaptation.parameter.value.double_value) + 0x9e3779b9 + (result << 6) + (result >> 2);
    result ^= bool_hash(adaptation.parameter.value.bool_value) + 0x9e3779b9 + (result << 6) + (result >> 2);
  
    return result;
    }
};
