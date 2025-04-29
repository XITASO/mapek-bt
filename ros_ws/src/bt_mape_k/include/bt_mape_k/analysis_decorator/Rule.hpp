#pragma once

#include "bt_mape_k/analysis_decorator/Trigger.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "bt_mape_k/analysis_decorator/AdaptationFactory.hpp"

#include <functional>
#include <string>

// Rule class: returns adaptation, if trigger condition is met
class Rule{
    public:
        Rule(std::string name, std::vector<std::string> components, std::shared_ptr<Trigger> trigger, Adaptation adaptation, bool on_change = true);

        // returns adaptation, if trigger condition is met
        std::optional<system_interfaces::msg::GenericAdaptation> execute() const;

        // returns adaptation, if trigger condition is met, and component matches
        std::optional<system_interfaces::msg::GenericAdaptation> execute(std::string component) const;

        // returns component
        std::vector<std::string> getComponents() const;

        // checks if the component is included in the list
        bool hasComponent(std::string component) const;

        std::string getName();

    private:
        mutable bool is_active = false;
        const bool on_change {};
        const std::shared_ptr<Trigger> trigger {};
        const Adaptation adaptation {};
        const std::vector<std::string> components;
        const std::string name {};
};