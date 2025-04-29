#include "bt_mape_k/analysis_decorator/RuleParser.hpp"
#include <fstream>
#include "bt_mape_k/util/string_utils.hpp"
#include <stdexcept>



RuleParser::RuleParser(BT::Blackboard::Ptr bbp):
blackboard_(bbp)
{}

/**
 * @brief Parses Adaptations for rule file 
 * 
 * first adds constants to blackboard
 * then parses rules
 * 
 * @param path the path to the rule file
 * 
 * @throws Runtime exception If the file input is unexpected
 * 
 * @return Vector of rules, pared from the file
 */
std::vector<std::shared_ptr<Rule>> RuleParser::parse(std::string path) const{
    std::vector<std::shared_ptr<Rule>> rules {};

    std::ifstream file;
    file.open(path); 

    if (!file.is_open())
        throw std::runtime_error( "did not find file at: "+ path);

    std::string line;

    // readconsts
    std::getline(file, line);
    if (line != "BEGIN CONSTS") throw std::runtime_error( "expected BEGIN CONSTS");
    while (true){
        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
        if (line == "END CONSTS") break;
        log_constant(line);
    }

    if (!std::getline(file, line)) std::runtime_error( "unexpected file ending");
    if (line != "BEGIN RULES") throw std::runtime_error( "expected BEGIN RULES");;

    // read rules
    while (true){
        
        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
        // returns (string | string) pair of rule name and rule type 
        auto name_type = parse_header(line);
        auto rule_name = name_type.first;
        auto on_change = name_type.second;

        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
        auto component_name = parse_components(line);

        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
        auto trigger = parse_triggers(line);

        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
        auto action = parse_action(line);

        // construct and add rule
        rules.push_back(std::make_shared<Rule>(rule_name, component_name, trigger, action, on_change));
        std::cout<<"added rule: '"<<rule_name<<std::endl;

        if (!std::getline(file, line)) std::runtime_error( "unexpected file ending");
        if (line == "END RULES") break;
    }
    file.close();
    return rules;
}

/**
 * @brief Adds constant to the black board
 * 
 * Reads the string and expects 3 words:
 * first word: type (bool, string, int, double)
 * second word: parameter name
 * third word: parameter value
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If an unexpected type is requested
 * @throws Runtime exception If the value cannot be casted
 * @throws Runtime exception If the number of words is unexpected
 * 
 */
void RuleParser::log_constant(std::string line) const{
    auto vec = string_utils::split(line);
    if (vec.size() != 3){
        throw std::runtime_error( line + "\n Expected 3 words in constant");
    }

    if (vec[0] == "bool"){
        std::cout<<"adding bool const: " << vec[1] <<" = "<<string_utils::to_bool(vec[2]) <<std::endl;
        blackboard_->set<bool>(vec[1].c_str(), string_utils::to_bool(vec[2]));
    } else if (vec[0] == "string"){
        std::cout<<"adding string const: " << vec[1] <<" = "<<vec[2] <<std::endl;
        blackboard_->set<std::string>(vec[1].c_str(), vec[2]);
    } else if (vec[0] == "double"){
         std::cout<<"adding double const: " << vec[1] <<" = "<<std::stod(vec[2]) <<std::endl;
         blackboard_->set<double>(vec[1].c_str(), std::stod(vec[2]));
    }else if (vec[0] == "int"){
         std::cout<<"adding int const: " << vec[1] <<" = "<<std::stoi(vec[2]) <<std::endl;
         blackboard_->set<int>(vec[1].c_str(), std::stoi(vec[2]));
    }
    
    else{
        throw std::runtime_error( vec[0] + "\n Expected 'double', 'int', 'string' or 'bool'");
    }
}

/**
 * @brief Parses the first line of a rule
 * 
 * Parses rule name and trigger type 
 * Reads the string and expects 2 or 3 words:
 * first word: keyword RULE
 * second word: rule name
 * third word: rule trigger type
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If an unexpected type is requested
 * @throws Runtime exception If the keyword 'RULE' is missing
 * @throws Runtime exception If an unexpected trigger type is requested
 * 
 */
std::pair<std::string, bool> RuleParser::parse_header(std::string line) const{
    auto vec = string_utils::split(line);
    if (vec[0] != "RULE"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'RULE'");
    }
    if (vec.size() < 2) throw std::runtime_error("every rule needs a name");
    
    auto name = vec[1];
    bool on_change = true;
    
    if (vec.size() == 3){
        if (vec[2] == "ON_TRIGGER_CHANGE") on_change = true;
        else if(vec[2] == "ON_EVERY_TRIGGER") on_change = false;
        else throw std::runtime_error(vec[2] + " unknown trigger type, expected: ON_TRIGGER_CHANGE or ON_EVERY_TRIGGER");
    }

    return {name, on_change};
}

/**
 * @brief Parses the second line of a rule
 * 
 * Parses rule components
 * Reads the string and expects at least
 * first word: keyword COMPONENTS
 * following words: components in brackes
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If nothing is enclosed in '[]'
 * @throws Runtime exception If the keyword 'COMPONENTS' is missing
 * 
 * @return vector of strings, the parsed components
 * 
 */
std::vector<std::string> RuleParser::parse_components(std::string line) const{
    auto wr = string_utils::read_word(line);
    if (wr.first != "COMPONENTS"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'COMPONENTS'");
    }
    //remove brackets
    auto vec = string_utils::find_in_brackets(wr.second,'[',']');
    if (vec.size() == 0){
        throw std::runtime_error( line + "\n Components must be enclosed in '[ ]', there must be atleast one component");
    }
    // get rid of commas
    vec = string_utils::split(string_utils::replace_all(vec[0], ',',' '));

    for (auto & w : vec) std::cout<<w<<std::endl;
    return vec;
}

/**
 * @brief Parses the third line of a rule
 * 
 * Parses rule components
 * Reads the string and expects at least
 * first word: keyword IF
 * following words: conditions in brackes '()'
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If more than one high level exists
 * @throws Runtime exception If the keyword 'IF' is missing
 * 
 * @return parsed trigger
 * 
 */
std::shared_ptr<Trigger>  RuleParser::parse_triggers(std::string line) const{
    auto wr = string_utils::read_word(line);
    if (wr.first != "IF"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'IF'");
    }

    // root condition, i.e. all that is in the outermost brackets
    auto in_brackets = string_utils::find_in_brackets(wr.second);
    if (in_brackets.size() != 1){
        throw std::runtime_error( "only one high level trigger allowed");
    }

    return parse_trigger(in_brackets[0]);
}

/**
 * @brief Recursively parses condition to generate trigger
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If differing logic operators are used within the samne trigger
 * @throws Runtime exception If unknown logic operators is used
 * 
 * @return parsed trigger
 * 
 */
std::shared_ptr<Trigger>  RuleParser::parse_trigger(std::string line) const{
    
    auto in_brackets = string_utils::find_in_brackets(line);
    auto between_brackets = string_utils::find_between_brackets(line);

    for (uint i = 1; i<between_brackets.size(); i++){
        if (between_brackets[i] != between_brackets[i-1])
            throw std::runtime_error( line + "\n Inside a sub-trigger the same logic operator is to be used");
    }

    std::vector<std::shared_ptr<Trigger>> sub_triggers;

    if (in_brackets.size()>0){
        
        for (const auto & sub_string : in_brackets)
            // recursion happens here
            sub_triggers.push_back(parse_trigger(sub_string));
        
        if (between_brackets[0] == "AND")
            return std::make_shared<Trigger>(sub_triggers, Trigger::AND);
        else if (between_brackets[0] == "OR")
            return std::make_shared<Trigger>(sub_triggers, Trigger::OR);
        else
            throw std::runtime_error( line + "\nonly 'AND and 'OR are allowed as logic operators");
    
    // here we are at leaf level
    } else{
        return parse_single_trigger(line);
    }
}

/**
 * @brief Parses as trigger condition 
 * 
 * Parses the trigger leaf lambda function 
 * The lambda function capture the black board and returns true or false, based on the condition
 * 
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If the number of word in the line is not 3
 * 
 * @throws Runtime exception If unknown Black board entry is used
 * @throws Runtime exception If invalid type comparison is used
 * @throws Runtime exception If type of values to compare does not match
 * 
 * @return parsed trigger
 * 
 */
std::shared_ptr<Trigger>  RuleParser::parse_single_trigger(std::string line) const{
    // TODO refactor into triggerfunc_factory 
    auto vec = string_utils::split(line);
    if (vec.size() != 3){
        throw std::runtime_error( line + "\n Expected 3 words in root trigger");
    }
    
    // lambda function for comparison
    auto bb = blackboard_;
    std::function<bool()> tf = [bb, vec](){

        auto entry_1 = bb->getEntry(vec[0]);
        auto entry_2 = bb->getEntry(vec[2]);

        if (!entry_1 ){
            std::cout<<"Key-entry not found: "<< vec[0] <<std::endl;
            return false;
        }

        if (!entry_2 ){
            std::cout<<"Key-entry not found: "<<vec[2]<<std::endl;
            return false;
        }
        
        
        auto type_id_1 = entry_1->info.typeName();
        auto type_id_2 = entry_2->info.typeName();

        // Note !! for a generic bb_entry of type string, the type becomes AnyTypeAllowed, thus they are treated equal
        if (type_id_1 == "AnyTypeAllowed") type_id_1 = "std::string";
        if (type_id_2 == "AnyTypeAllowed") type_id_2 = "std::string";

        if (type_id_1 != type_id_2){
            std::cout<<"Entry types do not match: "<< vec[0]<<" | "<<vec[2]<<" , "<<type_id_1 <<" | "<< type_id_2<<std::endl;
            return false;
        }

        if (type_id_1 == "std::string"){
            auto scoped_value1 = bb->get<std::string>(vec[0]);
            auto scoped_value2 = bb->get<std::string>(vec[2]);
            if (vec[1] == "==") return scoped_value1 == scoped_value2;
            if (vec[1] == "!=") return scoped_value1 != scoped_value2;
            std::cout<<"Unsupported operator for string type comparison: "<< vec[1]<<std::endl;
            return false;
        } else if (type_id_1 == "bool") {
            auto scoped_value1 = bb->get<bool>(vec[0]);
            auto scoped_value2 = bb->get<bool>(vec[2]);
            if (vec[1] == "==") return scoped_value1 == scoped_value2;
            if (vec[1] == "!=") return scoped_value1 != scoped_value2;
            std::cout<<"Unsupported operator for bool type comparison: "<< vec[1]<<std::endl;
            return false;
        } else if (type_id_1 == "double") {
            auto scoped_value1 = bb->get<double>(vec[0]);
            auto scoped_value2 = bb->get<double>(vec[2]);
            if (vec[1] == "==") return scoped_value1 == scoped_value2;
            if (vec[1] == "!=") return scoped_value1 != scoped_value2;
            if (vec[1] == ">=") return scoped_value1 >= scoped_value2;
            if (vec[1] == "<=") return scoped_value1 <= scoped_value2;
            if (vec[1] == ">")  return scoped_value1 > scoped_value2;
            if (vec[1] == "<")  return scoped_value1 < scoped_value2;
            std::cout<<"Unsupported operator for double type comparison: "<< vec[1]<<std::endl;
            return false;
        } else if (type_id_1 == "int") {
            auto scoped_value1 = bb->get<int>(vec[0]);
            auto scoped_value2 = bb->get<int>(vec[2]);
            if (vec[1] == "==") return scoped_value1 == scoped_value2;
            if (vec[1] == "!=") return scoped_value1 != scoped_value2;
            if (vec[1] == ">=") return scoped_value1 >= scoped_value2;
            if (vec[1] == "<=") return scoped_value1 <= scoped_value2;
            if (vec[1] == ">")  return scoped_value1 > scoped_value2;
            if (vec[1] == "<")  return scoped_value1 < scoped_value2;
            std::cout<<"Unsupported operator for int type comparison: "<< vec[1]<<std::endl;
            return false;
        }else{
            std::cout<<"Unsupported datatype: "<< type_id_1<<std::endl;
            return false;
        }
    };

    return std::make_shared<Trigger>(tf);
}

/**
 * @brief Parses the fourth line of the rule
 * 
 * Parses the Adaptation to trigger, using the adaptation factory
 * Reads the string and expects 2 or 4 words:
 * first word: keyword THEN
 * second word: adaptation type
 * third + fourth word: adaptation specification 
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If first word is not keyword "THEN"
 * 
 * @returns Adaptaion generating function
 * 
 */
Adaptation  RuleParser::parse_action(std::string line) const{
    auto wr = string_utils::read_word(line);
    if (wr.first != "THEN"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'THEN'");
    }
    auto vec = string_utils::split(wr.second);

    auto factory = AdaptationFactory();
    return factory.produce(vec, blackboard_);

}

