#pragma once

#include <iostream>
#include <utility>
#include <vector>

namespace string_utils{

// read first word of string; returns word and remainder
std::pair< std::string, std::string> read_word(const std::string & s, char delim = ' ');

// split string at delimiter, strips the string down first
std::vector<std::string> split(const std::string s, char delim = ' ');

// removes the delimiter in front and back
std::string strip(const std::string s, char delim = ' ');

// return an vector of all substrings enclosed in brackets
std::vector<std::string> find_in_brackets(const std::string & s, char b_opn = '(', char b_close = ')');

// return an vector of all substrings not enclosed  in brackets
std::vector<std::string> find_between_brackets(const std::string & s, char b_opn = '(', char b_close = ')');

// replaces all occurence of character a with character b
std::string replace_all(const std::string & s, char of, char with);

// returns bool from "true" or "false"
bool to_bool(const std::string & s);

// tries it best to convert to int
bool to_int(const std::string& input, int& output);

// tries it best to convert to double
bool to_double(const std::string& input, double& output);

// tries it best to convert to bool
bool to_bool(const std::string& input, bool& output);

//wrapper
std::string to_string(double);

//wrapper for bool, takes "true" or "false"
std::string to_string(bool);

//wrapper for std::string, just pipes through 
std::string to_string(std::string);

}