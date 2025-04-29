#include "bt_mape_k/util/logger.hpp"

std::shared_ptr<BTLogger> BTLogger::loggerPtr = std::shared_ptr<BTLogger>(nullptr);
std::mutex BTLogger::mtx;


/**
 * @brief Getter method for the global logger
 *
 * The method to retrieve the global logger, 
 * The method initializes the logger as a singleton if does not yet exist 
 *
 * @return Pointer to the globnal logger 
 */
std::shared_ptr<BTLogger> BTLogger::get_global_logger(){
    if (loggerPtr.get() == nullptr) {
        auto loggerPtr_ = new BTLogger();
        loggerPtr = std::shared_ptr<BTLogger>(loggerPtr_);
    }
    return loggerPtr;
}

/**
 * @brief Constructor of the global logger 
 *
 * Private to enable singelton
 *
 */
BTLogger::BTLogger():
    Node("BT_Logger"){
        logging_pub = this->create_publisher<system_interfaces::msg::ExperimentLogging>("/experiment_setup/bt_executor_log", 50);
}

/**
 * @brief returns the current time
 *
 * Uses ROS2 clock, can be synced with further ROS-Nodes for experimentations
 *
 * @return Current time in nano seconds
 */
long long BTLogger::get_time(){
    return this->get_clock()->now().nanoseconds();
}

/**
 * @brief Sends a logging message to the logging node
 * 
 * @param msg The message to be sent to the logging node
 */
void BTLogger::silent_global(system_interfaces::msg::ExperimentLogging* msg) const{
    //std::lock_guard<std::mutex> lock(pub_mtx);
    logging_pub->publish(*msg);
}

/**
 * @brief Logs to stdout as debug and sends a logging message to the logging node
 * 
 * @param log_output log_output the string to be logged to the terminal
 * @param logging_msg The message to be sent to the logging node
 */
void BTLogger::debug(std::string log_output, system_interfaces::msg::ExperimentLogging* logging_msg ) const{
    if(logging_msg){
        silent_global(logging_msg);
    }
    RCLCPP_DEBUG(this->get_logger(), log_output.c_str());
}

/**
 * @brief Logs to stdout as info and sends a logging message to the logging node
 * 
 * @param log_output log_output the string to be logged to the terminal
 * @param logging_msg The message to be sent to the logging node
 */
void BTLogger::info(std::string log_output, system_interfaces::msg::ExperimentLogging* logging_msg ) const{
    if(logging_msg){
        silent_global(logging_msg);
    }
    RCLCPP_INFO(this->get_logger(), log_output.c_str());
}

/**
 * @brief Logs to stdout as waring and sends a logging message to the logging node
 * 
 * @param log_output log_output the string to be logged to the terminal
 * @param logging_msg The message to be sent to the logging node
 */
void BTLogger::warn(std::string log_output, system_interfaces::msg::ExperimentLogging* logging_msg ) const{
    if(logging_msg){
        silent_global(logging_msg);
    }
    RCLCPP_WARN(this->get_logger(), log_output.c_str());
}

/**
 * @brief Logs to stdout as error and sends a logging message to the logging node
 * 
 * @param log_output log_output the string to be logged to the terminal
 * @param logging_msg The message to be sent to the logging node
 */
void BTLogger::error(std::string log_output, system_interfaces::msg::ExperimentLogging* logging_msg ) const{
    if(logging_msg){
        silent_global(logging_msg);
    }
    RCLCPP_ERROR(this->get_logger(), log_output.c_str());
}

/**
 * @brief Logs to stdout as fatal and sends a logging message to the logging node
 * 
 * @param log_output log_output the string to be logged to the terminal
 * @param logging_msg The message to be sent to the logging node
 */
void BTLogger::fatal(std::string log_output, system_interfaces::msg::ExperimentLogging* logging_msg ) const{
    if(logging_msg){
        silent_global(logging_msg);
    }
    RCLCPP_FATAL(this->get_logger(), log_output.c_str());
}
