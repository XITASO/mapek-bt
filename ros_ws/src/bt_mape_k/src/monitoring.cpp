#include <bt_mape_k/monitoring.hpp>
using namespace BT;

/**
 * @brief Implements the behavior tree node's tick action for monitoring and updating the blackboard based on
 *        incoming messages.
 * 
 * @param msg A shared pointer to a SetBlackboardGroup message containing parameters to update on the blackboard.
 * 
 * @return NodeStatus indicating the result of the action execution, returns NodeStatus::SUCCESS if the operation
 *         completes successfully, or NodeStatus::FAILURE if there's an issue with accessing the blackboard.
 * 
 * The onTick function retrieves the current time and sets it as an output on the "clock" port of the node.
 * If a message is provided, it iterates over the message's parameters, logs the parameter details for debugging
 * purposes, and sets these values on the blackboard based on their data types (bool, integer, double, or string).
 * The function checks if a blackboard object is available; otherwise, it logs an error and returns a failure status.
 */
NodeStatus Monitoring::onTick(const std::shared_ptr<system_interfaces::msg::SetBlackboardGroup> &msg)
{
  // Access the current time
  rclcpp::Time current_time = node_handle->now();

  // Convert the time to milliseconds
  int current_time_ms = current_time.nanoseconds() / 1e6;
  setOutput("clock", current_time_ms);
  //RCLCPP_INFO(logger(), "New tick started now");

  if (msg) // empty if no new message received, since the last tick
  {
    for (rcl_interfaces::msg::Parameter param_msg : msg->bb_params)
    {
      RCLCPP_DEBUG(logger(), "[%s] new message: %s, %i", name().c_str(),
                  param_msg.name.c_str(), param_msg.value.type);
    }
  }
  else
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Directly access the blackboard.
  if (blackboard_)
  {
    for (rcl_interfaces::msg::Parameter param_msg : msg->bb_params)
    {
      auto parameter_value = param_msg.value;
      switch (parameter_value.type)
      {
      case static_cast<int>(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL):
        blackboard_->set<bool>(param_msg.name.c_str(), parameter_value.bool_value);
        RCLCPP_DEBUG(logger(), "Bool: %s", parameter_value.bool_value ? "true" : "false");
        break;
      case static_cast<int>(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER):
        blackboard_->set<int>(param_msg.name.c_str(), parameter_value.integer_value);
        // RCLCPP_DEBUG(logger(), "Integer: %ld", parameter_value.integer_value);
        RCLCPP_DEBUG(logger(), "Set blackboard entry: %s to %i", param_msg.name.c_str(), blackboard_->get<int>(param_msg.name.c_str()));
        break;
      case static_cast<int>(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE):
        blackboard_->set<double>(param_msg.name.c_str(), parameter_value.double_value);
        RCLCPP_DEBUG(logger(), "Setting blackboard entry: %s to %f", param_msg.name.c_str(), parameter_value.double_value);
        break;
      case static_cast<int>(rcl_interfaces::msg::ParameterType::PARAMETER_STRING):
        blackboard_->set<std::string>(param_msg.name.c_str(), parameter_value.string_value.c_str());
        RCLCPP_DEBUG(logger(), "String: %s", parameter_value.string_value.c_str());
        break;
      default:
        RCLCPP_WARN(logger(), "Unknown parameter type");
      }
    }
  }
  else
  {
    std::cerr << "Error: Blackboard is not available" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}