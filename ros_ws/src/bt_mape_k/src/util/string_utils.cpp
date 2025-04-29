#include "bt_mape_k/util/string_utils.hpp"

namespace string_utils{

/**
 * @brief Reads the first word of a given string
 * 
 * @param s The string to be read
 * @param delim The delimiter that separates the words (default ' ')
 * 
 * @return a pair of strings: the read word and the remnaining string 
 */
    std::pair< std::string, std::string> read_word(const std::string & s, char delim){
        auto s1 = strip(s);
        int i = s1.find(delim);

        if (i <= 0){
            return {s1, ""};
        }

        auto remainder =s1.substr(i+1);
        auto word = s1.substr(0,i);
        return {word, remainder};
    }

/**
 * @brief Split a given string into words
 * 
 * @param s The string to be read
 * @param delim The delimiter that separates the words (default ' ')
 * 
 * @return a vecotr of strings: all words contained in the string 
 */
    std::vector<std::string> split(std::string s, char delim){
        std::vector<std::string> vec;
        while(s.size()){
            auto wr = read_word(s, delim);
            vec.push_back(wr.first);
            s = wr.second;
        }
        return vec;
    }

 /**
 * @brief Finds substrings that are enclosed in brackets
 * 
 * On call, all brackets on the first layer are considered, thus the returned
 * string might contain brackets as well 
 * 
 * @param s The string to be read
 * @param b_opn The open bracket character (default '(')
 * @param b_close The closing bracket character (default ')')
 * 
 * @return vector of strings: all string enclosed in brackets 
 */
    std::vector<std::string> find_in_brackets(const std::string & s, char b_opn, char b_close){
        int bracket_ctr = 0;
        bool f = false;
        std::vector<std::string> in_brackets{};

        std::string current_word{};
        for (uint i = 0; i <s.length(); i++){

            if (s[i] == b_opn){
                bracket_ctr += 1;
                f = true;
                if (bracket_ctr == 1)
                    continue;
            }

            if (s[i] == b_close){
                bracket_ctr -=1;
                if (bracket_ctr == 0){
                    in_brackets.push_back(current_word);
                    f = false;
                    current_word  ="";
                }
            }

            if (f) {
                current_word += s[i];
            }
        }
        return in_brackets;
    }

/**
 * @brief Finds substrings that are not enclosed in brackets
 * 
 * @param s The string to be read
 * @param b_opn The open bracket character (default '(')
 * @param b_close The closing bracket character (default ')')
 * 
 * @return vector of strings: all string not enclosed in brackets 
 */
    std::vector<std::string> find_between_brackets(const std::string & s, char b_opn, char b_close){
        int bracket_ctr = 0;
        bool f = false;

        std::vector<std::string> in_brackets{};

        std::string current_word{};
        for (uint i = 0; i <s.length(); i++){

            if (s[i] == b_opn){
                bracket_ctr += 1;
                f = true;
                continue;
            }

            if (s[i] == b_close){
                bracket_ctr -=1;
                if (bracket_ctr == 0){
                    if (current_word != "")
                        in_brackets.push_back(strip(current_word));
                    f = false;
                    current_word = "";
                    continue;
                }
            }

            if (!f) {
                current_word += s[i];
            }
        }
        return in_brackets;
    }

/**
 * @brief Removes all instances of a given character from the begin and end of a string  
 * 
 * @param s The string to be read
 * @param delim The character to be removed
 * 
 * @return the stripped string
 */
    std::string strip(const std::string s, char delim ){
        uint i = s.find_first_not_of(delim);
        uint j = s.find_last_not_of(delim);

        if (i == s.length()-1 and s[0] == delim) return ""; 

        return s.substr(i,j-i+1);
    }

/**
 * @brief Replaces all instances of a given character with another one  
 * 
 * @param s The string to be read
 * @param of The character to be replaced
 * @param of The character it should be replaced with
 * 
 * @return the string with replaced characters
 */
    std::string replace_all(const std::string & s, char of, char with){
        auto str_copy = s;
        for (auto& c : str_copy){
            if (c == of) c = with;
        }
        return str_copy;
    }

/**
 * @brief parses a string to bool
 *  true if the string is "true"
 *  false if the string is "false"
 * 
 * @param s The string to be read
 * 
 * @throw RuntimeError If the string is neither "true" nor "false"
 * 
 * @return boolean corresponding to the string
 */
    bool to_bool(const std::string & s){
        if (s=="false") return false;
        else if  (s=="true") return true;
        else throw::std::runtime_error("expected 'true' or 'false'");
    }

    std::string to_string(double val){
        return std::to_string(val);
    }

/**
 * @brief returns "true" or "false" corresponding to a given bool

 * @param val The bool to be converted
 * 
 * @return string "true" or "false"
 */
    std::string to_string(bool val){
        if (val) return "true";
        return "false";
    }

/**
 * @brief This function does nothing, wrapper to be used for abstract implementations

 * @param val input
 * 
 * @return input as well
 */
    std::string to_string(std::string val){
        return val;
    }

/**
 * @brief converts string to an int
 * 
 * wraps std:stio
 * 
 * @param[in] input the string to be cast
 * @param[out] output the corresponding int
 * 
 * @return bool Indicator if the cast was successful
 */
    bool to_int(const std::string& input, int& output) {
        try {
            size_t pos;
            output = std::stoi(input, &pos);
            if (pos == input.length()) {
                return true;
            }
        } catch (const std::invalid_argument& e) {
            // Not an int
        } catch (const std::out_of_range& e) {
            // Out of int range
        }
        return false;
    }

/**
 * @brief converts string to a double
 * 
 * wraps std:stod
 * 
 * @param[in] input the string to be cast
 * @param[out] output the corresponding double
 * 
 * @return bool Indicator if the cast was successful
 */
    bool to_double(const std::string& input, double& output) {
        try {
            size_t pos;
            output = std::stod(input, &pos);
            if (pos == input.length()) {
                return true;
            }
        } catch (const std::invalid_argument& e) {
            // Not a double
        } catch (const std::out_of_range& e) {
            // Out of double range
        }
        return false;
    }

    /**
 * @brief converts string to bool
 * 
 * check if the string is "true" or "false" and return the corresponding value as bool
 * 
 * @param[in] input the string to be cast
 * @param[out] output the corresponding bool
 * 
 * @return bool Indicator if the cast was successful
 */
    bool to_bool(const std::string& input, bool& output) {
        if (input=="true"){
            output = true;
            return true;
        }
        if (input=="false"){
            output = false;
            return true;
        }
        return false;
    }

}