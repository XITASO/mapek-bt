#pragma once

#include <string>
#include <unordered_map>

#include "bt_mape_k/analysis_decorator/Rule.hpp"
#include "bt_mape_k/analysis_decorator/Trigger.hpp"
#include "bt_mape_k/analysis_decorator/AdaptationFactory.hpp"

#include "behaviortree_cpp/blackboard.h"

// helper class to handle rule file
class RuleParser{
    public:
        RuleParser(BT::Blackboard::Ptr);

        // parses the file to a list of rules and sets constants
        std::vector<std::shared_ptr<Rule>> parse(std::string path) const;
    private:
        // parses first line of rule, rule name and activation type
        std::pair<std::string, bool> parse_header(std::string) const;
        
        // parses second line of rule, component
        std::vector<std::string> parse_components(std::string) const;

        // parses third line of rule, trigger
        std::shared_ptr<Trigger> parse_triggers(std::string) const;

        // recursively parses triggers
        std::shared_ptr<Trigger> parse_trigger(std::string) const;

        // parses trigger leaf functions
        std::shared_ptr<Trigger> parse_single_trigger(std::string) const;

        // parses fourth line of rule, adaptation by using Adaptation factory
        Adaptation parse_action(std::string) const;

        // parses constant and writes to bb
        void log_constant(std::string) const;

        
        BT::Blackboard::Ptr blackboard_;
};