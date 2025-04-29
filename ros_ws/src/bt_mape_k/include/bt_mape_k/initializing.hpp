#include "behaviortree_cpp/bt_factory.h"
#include "bt_mape_k/json.hpp"
#include <fstream>

#include <string>
#include <unordered_map>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>

#define CAMERA_SENSOR "/managed_subsystem/camera"
#define LIDAR_SENSOR "/managed_subsystem/lidar"
#define IMAGE_ENHANCEMENT "/managed_subsystem/image_enhancement"
#define SENSOR_FUSION "/managed_subsystem/sensor_fusion"
#define SEGMENTATION "/managed_subsystem/segmentation"


// Define a graph using an adjacency list
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

using namespace BT;

class InitializeBlackboard: public SyncActionNode
{
public:
    InitializeBlackboard(const std::string& name, const NodeConfiguration& config, std::string file_path)
        : SyncActionNode(name, config), blackboard_(config.blackboard),file_path(file_path) {}

    static PortsList providedPorts()
    {
        return {
            OutputPort<std::pair<Graph,std::unordered_map<std::string, std::pair<Vertex, bool>>>>("dependency_graph")
        };
    }

    NodeStatus tick() override;

    private:
        Blackboard::Ptr blackboard_;
        Graph g;
        // Map to store vertex descriptors associated with node names
        std::unordered_map<std::string, std::pair<Vertex, bool>> vertex_map;
        void addVertexWithName(const std::string& name);
        std::string file_path;
};