cmake_minimum_required(VERSION 3.8)
project(bt_mape_k)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB SRC "src/*.cpp" "src/*/*.cpp")
add_executable(bt_executor ${SRC})

# add_executable(bt_executor 
#   src/bt_executor.cpp 
#   src/monitoring.cpp 
#   src/initializing.cpp 
#   src/planning.cpp
#   src/analysing.cpp
#   src/analysis_decorator/initializing.cpp
#   src/analysis_decorator/initializing.cpp
#   )

ament_target_dependencies(bt_executor behaviortree_cpp behaviortree_ros2 system_interfaces std_msgs)

target_include_directories(bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bt_executor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS bt_executor 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
