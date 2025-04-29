#include <bt_mape_k/initializing.hpp>

/**
 * @brief Implements the behavior tree node's tick action to initialize the blackboard with data from a JSON file
 *        and build a directed graph representing component dependencies.
 * 
 * @return NodeStatus indicating the execution result, returns NodeStatus::SUCCESS if initialization completes
 *         successfully, or NodeStatus::FAILURE if file access encounters an error.
 * 
 * The tick function reads data from a specified JSON file, parsing it to set initial values on the blackboard
 * (e.g., strings, integers, floats, booleans). It constructs a dependency graph by adding vertices for system
 * components and edges representing dependencies between them. Upon successful execution, the dependency graph
 * and vertex mapping are sent as output for subsequent nodes to utilize. Error handling occurs if file operations
 * fail, logging relevant messages for troubleshooting.
 */
NodeStatus InitializeBlackboard::tick()
{
    // Create an ifstream object to read from the file
    std::ifstream inputFile(file_path);

    // Check if the file was opened successfully
    if (!inputFile.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Create a JSON object
    nlohmann::json jsonData;

    // Parse the JSON data from the file
    inputFile >> jsonData;

    // Close the file
    inputFile.close();

    for (const auto& [key, value] : jsonData.items()) {
        if (value.is_string())
        {
            std::string str = value.get<std::string>();
            blackboard_->set<std::string>(key, str);
        }
        if (value.is_number_integer()) {
            int num = value.get<int>();
            blackboard_->set<int>(key, num);
        }
        else if (value.is_number_float()) {
            double num = value.get<double>();
            blackboard_->set<double>(key, num);
        } 
        if (value.is_boolean()) {
            bool bool_ = value.get<bool>();
            blackboard_->set<bool>(key, bool_);
        }
    }

    return NodeStatus::SUCCESS;
 
}