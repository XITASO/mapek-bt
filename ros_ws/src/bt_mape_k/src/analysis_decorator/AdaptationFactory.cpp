#include "bt_mape_k/analysis_decorator/AdaptationFactory.hpp"
#include "bt_mape_k/util/adaptation_utils.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

#include <stdexcept>

/**
 * @brief Creates a adaptation from a given vecor of strings
 * 
 *  the first entry of the vector holds the adaption's type 
 *  the second entry of the vector holds the adaption's parameter name
 *  the third entry of the vector holds the adaption's parameter value as blackboard key
 * 
 * @param vec vector of strings instructions on how to build the adaptation
 * @param bb blackboard pointerm to be captured by the adaptaion lambda
 * 
 * @return Adaptation Lambda function that returns the adaptation with the current value
 */
Adaptation AdaptationFactory::produce(std::vector<std::string> vec, BT::Blackboard::Ptr bb){
    auto type = adaptations_utils::deduceAdaptationType(vec[0]);
    if (   type == system_interfaces::msg::AdaptationType::ACTION_ACTIVATE
        || type == system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE
        || type == system_interfaces::msg::AdaptationType::ACTION_RESTART
        || type == system_interfaces::msg::AdaptationType::ACTION_REDEPLOY
    ) return produceNoParamAdaptation(type);

    return produceParametrizedAdaptation(type, vec, bb);
}


/**
 * @brief Creates a adaptation without parameters
 * 
 * @param type type of the adaptation
 * 
 * @return Adaptation Lambda function that statically returns the adaptation of a given type
 */
Adaptation AdaptationFactory::produceNoParamAdaptation(uint8_t type){
    return [type]()-> system_interfaces::msg::GenericAdaptation {
        system_interfaces::msg::GenericAdaptation msg;
        msg.action = type;
        return msg;
    };
}

/**
 * @brief Creates a adaptation with parameters
 * 
 * builds adynamic adaptation function from a string vector:
 * the second entry of the vector holds the adaption's parameter name
 * the third entry of the vector holds the adaption's parameter value as blackboard key
 * 
 * @param type type of the adaptation
 * @param vec vector of strings instructions on how to build the adaptation
 * @param bb blackboard pointerm to be captured by the adaptaion lambda
 * 
 * @return Adaptation Lambda function that dynamically fills the adaptation value with the current blackboard entry
 */
Adaptation AdaptationFactory::produceParametrizedAdaptation(uint8_t type, std::vector<std::string> vec, BT::Blackboard::Ptr bb){
    if (vec.size()!=3)
        throw std::runtime_error( "expected three arguments in adaptation");

    return [type, vec, bb, this]()-> system_interfaces::msg::GenericAdaptation {

        auto bb_key = vec[2];
        auto para_name = vec[1];

        auto bb_entry = bb->getEntry(bb_key);
        if (!bb_entry) throw std::runtime_error("unknwon bb key: "+bb_key);
        auto type_name = bb_entry->info.typeName();

        // std::string value_str{};

        system_interfaces::msg::GenericAdaptation msg;
        msg.action = type;
        msg.parameter.name = para_name;

        if (type_name == "std::string" || type_name == "AnyTypeAllowed"){
            if (type != system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER && type != system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION)
                throw std::runtime_error ("for string-type parameters only 'set' action is supported");
            msg.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            msg.parameter.value.string_value = bb->get<std::string>(bb_key);
        }else if(type_name == "bool"){
            if (type != system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER)
                throw std::runtime_error ("for bool-type parameters only 'set' action is supported");
            msg.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            msg.parameter.value.bool_value = bb->get<bool>(bb_key);
        }else if (type_name == "double"){
            // for double every thing is allowed, nice
            msg.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            msg.parameter.value.double_value = bb->get<double>(bb_key);
        }else if (type_name == "int"){
            // for double every thing is allowed, nice
            msg.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            msg.parameter.value.integer_value = bb->get<int>(bb_key);
        }
        else{
            throw std::runtime_error ("unsupported datatype: "+type_name);
        }



        return msg;
    };
}

