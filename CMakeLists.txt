cmake_minimum_required(VERSION 3.8)
project(mtsp_drone_delivery_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)


### NOTE if you want to add python libs add them here
ament_python_install_package(python_lib)
### NOTE

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)





include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(swarm_planner 
	src/swarm_planner.cpp
	src/state_validity_checker.cpp
	src/swarm_config_tracker.cpp
)




### NOTE add Cpp nodes
add_executable(swarm_planner_node src/swarm_planner_node.cpp)

target_link_libraries(swarm_planner_node
  swarm_planner
  ${OMPL_LIBRARIES}
)

target_include_directories(swarm_planner_node PUBLIC

### NOTE you can add additional include dirs for Cpp here
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  $<INSTALL_INTERFACE:include>)
target_compile_features(swarm_planner_node PUBLIC c_std_99 cxx_std_17 cxx_std_20)  # Require C99 and C++17

### NOTE add dependencies to the cpp target
ament_target_dependencies(swarm_planner_node
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
  "Eigen3"
)
### NOTE


install(TARGETS swarm_planner_node
  DESTINATION lib/${PROJECT_NAME})


### NOTE add python nodes
install(PROGRAMS
	scripts/node_py.py
	DESTINATION lib/${PROJECT_NAME})

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
