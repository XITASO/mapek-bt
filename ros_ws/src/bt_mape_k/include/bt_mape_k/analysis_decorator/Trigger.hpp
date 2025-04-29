#pragma once

#include <functional>
#include <vector>
#include <memory>

// recursive check function to represent nested logic
class Trigger{
public:

    enum LogicOperator{
        AND= 0,
        OR,
        NOT,
        ND
    };

    // ctor for leaf level
    Trigger( std::function<bool()> trigger_function);

    // ctor for tree level
    Trigger( std::vector<std::shared_ptr<Trigger>> childs, LogicOperator logic);

    // checks condition, is called recursively
    bool check();

private:
    std::function<bool()> trigger_function {};
    bool is_leaf = false;
    std::vector<std::shared_ptr<Trigger>> children;
    LogicOperator logic = ND;

};