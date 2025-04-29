#include "behaviortree_cpp/bt_factory.h"
// code is mainly copied from https://github.com/BehaviorTree/BehaviorTree.CPP/blob/14589e589c5049fbb7f1fb30ccaa3efc66c84f18/include/behaviortree_cpp/actions/script_condition.h
// class is extended with a memory function. Could not inherit from the class since the tick function there is private.

using namespace BT;

class ScriptConditionWithMemory : public ConditionNode
{
public:
  ScriptConditionWithMemory(const std::string& name, const NodeConfig& config)
    : ConditionNode(name, config)
  {
    loadExecutor();
  }

  static PortsList providedPorts()
  {
    return { InputPort("code", "Piece of code that can be parsed. Must return false or "
                               "true") };
  }

private:
  virtual BT::NodeStatus tick() override
  {
    loadExecutor();
    
    Ast::Environment env = { config().blackboard, config().enums };
    auto result = _executor(env);
    NodeStatus current_status = (result.cast<bool>()) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    
    // Only change the status if the condition state changes
    if (current_status != _last_status)
    {
      _last_status = current_status; // remember the last status
      return current_status;
    }
    
    // If the condition hasn't changed, return IDLE or keep the last status 
    // You might choose to return IDLE or keep returning the same status as before
    return NodeStatus::SUCCESS;
  }

  void loadExecutor()
  {
    std::string script;
    if(!getInput("code", script))
    {
      throw RuntimeError("Missing port [code] in ScriptCondition");
    }
    if(script == _script)
    {
      return;
    }
    auto executor = ParseScript(script);
    if(!executor)
    {
      throw RuntimeError(executor.error());
    }
    else
    {
      _executor = executor.value();
      _script = script;
    }
  }

  std::string _script;
  ScriptFunction _executor;
  NodeStatus _last_status;  // To store the last evaluated condition status
};

