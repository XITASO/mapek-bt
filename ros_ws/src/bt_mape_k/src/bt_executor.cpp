#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "bt_mape_k/analysing.hpp"
#include "bt_mape_k/planning.hpp"
#include "bt_mape_k/register_adaptation.hpp"
#include "bt_mape_k/monitoring.hpp"
#include "bt_mape_k/execution.hpp"
#include "bt_mape_k/initializing.hpp"
#include "bt_mape_k/script_condition_memory.hpp"

#include <thread>

using namespace BT;

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick()
  {
    auto msg = getInput<float>("text");
    if(!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<float>("text") };
  }
};

int main(int argc, char** argv){

  std::string file_folder = "./src/bt_mape_k/bts/";

  std::string rule_set = "rules.txt";
  if (argc>=2){
    rule_set = argv[1];
  }

  std::string rules_path = file_folder+rule_set;
  std::string init_path =  file_folder+"init_blackboard.json";
  std::string bt_path =    file_folder+"main_bt.xml";

  std::cout<<rules_path<<std::endl;

  rclcpp::init(argc, argv);
  auto nh_monitoring = std::make_shared<rclcpp::Node>("bt_monitoring_subscriber");
  BehaviorTreeFactory factory;

  // Monitoring
  RosNodeParams monitoring_ros_node_params;
  monitoring_ros_node_params.nh = nh_monitoring;
  monitoring_ros_node_params.default_port_value = "blackboard_group";
  factory.registerNodeType<Monitoring>("Monitoring", monitoring_ros_node_params);
  factory.registerNodeType<InitializeBlackboard>("InitializeBlackboard", init_path);
  factory.registerNodeType<SaySomething>("SaySomething");

  // Analysing and Planning
  factory.registerNodeType<ScriptConditionWithMemory>("ScriptConditionWithMemory");
  factory.registerNodeType<AnalysisDecorator>("AnalysisDecorator", rules_path);
  factory.registerNodeType<RegisterAdaptation>("RegisterAdaptation");
  factory.registerNodeType<PlanningDecorator>("PlanningDecorator");

  // Executing
  RosNodeParams lifecycle_ros_node_params;
  RosNodeParams parametrization_ros_node_params;
  RosNodeParams comm_change_ros_node_params;
  RosNodeParams redeploy_ros_node_params;
  auto nh_lifecycle = std::make_shared<rclcpp::Node>("bt_execution_lifecycle");
  auto nh_parametrization = std::make_shared<rclcpp::Node>("bt_execution_parametrization");
  auto nh_comm_change = std::make_shared<rclcpp::Node>("bt_execution_comm_change");
  auto nh_redeploy = std::make_shared<rclcpp::Node>("bt_execution_redeploy");

  lifecycle_ros_node_params.nh = nh_lifecycle;
  parametrization_ros_node_params.nh = nh_parametrization;
  comm_change_ros_node_params.nh = nh_comm_change;
  redeploy_ros_node_params.nh = nh_redeploy;

  factory.registerNodeType<ExecuteCommChange>("ExecuteCommunicationAdaptation", comm_change_ros_node_params);
  factory.registerNodeType<ExecuteLifeCycle>("ExecuteLifecycle", lifecycle_ros_node_params);
  factory.registerNodeType<ExecuteParametrization>("ExecuteParametrization", parametrization_ros_node_params);
  factory.registerNodeType<ExecuteRedeploy>("ExecuteRedeploy", redeploy_ros_node_params);

  factory.registerBehaviorTreeFromFile(bt_path); 

  auto main_tree = factory.createTree("Main");

  std::chrono::milliseconds intervall_time = std::chrono::milliseconds(200);

  while(rclcpp::ok())
  {
    auto tick_start_time = std::chrono::steady_clock::now();
    main_tree.tickWhileRunning(intervall_time);
    
    // Calculate sleep time
    auto tick_end_time = std::chrono::steady_clock::now();
    auto tick_duration = std::chrono::duration_cast<std::chrono::milliseconds>(tick_end_time - tick_start_time);
    std::chrono::milliseconds sleep_time = intervall_time - tick_duration;

    // Only sleep if the tick duration was less than 100ms
    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);
    }
    else {
      std::cout << "[Warning] BT frequency is too high, execution took longer than expected." << std::endl;
    }
  }

  return 0;
}