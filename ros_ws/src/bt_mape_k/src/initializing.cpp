#include <bt_mape_k/initializing.hpp>

/**
 * @brief Adds a named vertex to the directed graph structure used for managing dependencies within the system.
 * 
 * @param name A string representing the name of the vertex to be added to the graph.
 * 
 * This function creates a new vertex within the graph and associates it with the given name. 
 * The vertex is stored in the vertex_map for future access and manipulation implementing the dependency graph used 
 * in the MAPE-K loop
 */
void InitializeBlackboard::addVertexWithName(const std::string& name) {
        Vertex v = boost::add_vertex(g);
        vertex_map[name] = std::make_pair(v, false);
    }

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

    // Add nodes (vertices)
    addVertexWithName(CAMERA_SENSOR);
    addVertexWithName(LIDAR_SENSOR);
    addVertexWithName(IMAGE_ENHANCEMENT);
    addVertexWithName(SENSOR_FUSION);
    addVertexWithName(SEGMENTATION);

    // Add edges (directed)
    boost::add_edge(vertex_map[IMAGE_ENHANCEMENT].first, vertex_map[CAMERA_SENSOR].first, g);
    boost::add_edge(vertex_map[SENSOR_FUSION].first, vertex_map[CAMERA_SENSOR].first, g);
    boost::add_edge(vertex_map[SENSOR_FUSION].first, vertex_map[LIDAR_SENSOR].first, g);
    boost::add_edge(vertex_map[SENSOR_FUSION].first, vertex_map[IMAGE_ENHANCEMENT].first, g);
    boost::add_edge(vertex_map[SEGMENTATION].first, vertex_map[SENSOR_FUSION].first, g);

    setOutput("dependency_graph", std::make_pair(g, vertex_map));
       

    return NodeStatus::SUCCESS;
 
}