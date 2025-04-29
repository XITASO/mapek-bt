#include "bt_mape_k/analysis_decorator/Trigger.hpp"

#include <iostream>

    /**
     * @brief Constructor of the trigger class
     * 
     * This constructor creates a leaf node trigger
     * 
     * @param trigger_function function to be checked
     */
    Trigger::Trigger( std::function<bool()> trigger_function):
        trigger_function(trigger_function),
        is_leaf(true){}
    
    /**
     * @brief Constructor of the trigger class
     * 
     * This constructor creates a not leaf node trigger
     * 
     * @param children sub trigger, i.e. subcondition to be checked
     * @param logic the logic operator (AND, OR) with wich the children are linked
     */
    Trigger::Trigger( std::vector<std::shared_ptr<Trigger>> children, LogicOperator logic) : 
        children(children),
        logic(logic),
        is_leaf(false){}
        
    
    /**
     * @brief Checks Trigger Status 
     * 
     * Recursively checks all child and respectively the trigger functions, if the trigger condition is true
     * 
     * @return bool, the result of the trigger evaluation
     */    
    bool Trigger::check(){
        if (is_leaf)
            return this->trigger_function();

        switch (logic){
            case AND:
            {
                for (const auto & child : children){
                    if (!child->check()) return false;
                }
                return true;
            }
            case OR:
            {
                for (const auto & child : children){
                    if (child->check()) return true;
                }
                return false;
            }
            default:
            {
                std::cout<<"unsuitable logic operation disabling\n";
                return false;
            }
        }
    }