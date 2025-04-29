#include "bt_mape_k/analysis_decorator/Rule.hpp"
#include <iostream>
#include <optional>

/**
 * @brief Constructor of the Rule class
 * 
 * @param name The name of the rule
 * @param components All components the rule applies to
 * @param trigger The trigger to be checked
 * @param adaptation The adaptation top be execute if triggered
 * @param on_change execution policy: if true execute the adaptation only if the trigger state changes,
 * if false execute on every trigger
 * 
 */
Rule::Rule(std::string name, std::vector<std::string> components, std::shared_ptr<Trigger> trigger, Adaptation adaptation, bool on_change):
    on_change(on_change),
    trigger(trigger),
    components(components),
    adaptation(adaptation),
    name(name)
    {
        auto adaptations = std::vector<system_interfaces::msg::GenericAdaptation>{};
    }

/**
 * @brief Executes the rule
 * 
 * Checks the trigger and if is evauates to "true", it returns the corresponding adaptation
 * 
 * @return Optional Adaptation contains the adaptation if triggered
 */
std::optional<system_interfaces::msg::GenericAdaptation> Rule::execute() const{
    if(trigger->check()){
        if(!is_active || !on_change) {
            is_active = true;
            return adaptation();
        } 
    }else{
        is_active = false;
    }
    return{};
}

/**
 * @brief Getter for the rule name
 * 
 * @return string The name of the rule
 */
std::string Rule::getName()
{
    return name;
}

/**
 * @brief Executes the rule
 * 
 * Checks the trigger and if is evauates to "true" AND the rule applied to the given component, it returns the corresponding adaptation 
 * 
 * @param component the name of the component to filter by
 * 
 * @return Optional Adaptation contains the adaptation if triggered
 */
std::optional<system_interfaces::msg::GenericAdaptation> Rule::execute(std::string component) const{
    if(hasComponent(component)){
        return execute();
    }
    return{};
}

bool Rule::hasComponent(std::string query) const{
    for (const auto& component : components){
        if (query == component)
            return true;
    }
    return false;
}

/**
 * @brief Getter for the rules components
 * 
 * @return vector of the components the rules applies to
 */
std::vector<std::string> Rule::getComponents() const{
    return components;
}