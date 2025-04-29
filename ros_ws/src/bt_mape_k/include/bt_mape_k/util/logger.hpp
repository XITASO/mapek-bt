#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <system_interfaces/msg/experiment_logging.hpp>

class BTLogger : public rclcpp::Node{
    private:

        static std::shared_ptr<BTLogger> loggerPtr;
        static std::mutex mtx;

        mutable std::mutex pub_mtx {};

        // Private Constructor, so noone can create a new separate logger
        BTLogger();

        rclcpp::Publisher<system_interfaces::msg::ExperimentLogging>::SharedPtr logging_pub;
        system_interfaces::msg::ExperimentLogging message;

        
    
    public:
        // Deleting the copy constructor to prevent copies
        BTLogger(const BTLogger& obj) = delete;
    
        // Static method to get the Singleton instance, awkward name to not shadow the get_logger of ros node
        static std::shared_ptr<BTLogger> get_global_logger();

        // get time of underlying ros node
        long long get_time(); 

        // publish message
        void silent_global(system_interfaces::msg::ExperimentLogging*) const;

        // ros debug comment, publish message, if provided
        void debug(std::string comment, system_interfaces::msg::ExperimentLogging* = nullptr) const;
        // ros info comment, publish message, if provided
        void info( std::string comment, system_interfaces::msg::ExperimentLogging* = nullptr) const;
        // ros warn comment, publish message, if provided
        void warn( std::string comment, system_interfaces::msg::ExperimentLogging* = nullptr) const;
        // ros error comment, publish message, if provided
        void error(std::string comment, system_interfaces::msg::ExperimentLogging* = nullptr) const;
        // ros fatal comment, publish message, if provided
        void fatal(std::string comment, system_interfaces::msg::ExperimentLogging* = nullptr) const;

    };