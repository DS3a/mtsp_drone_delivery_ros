cmake_minimum_required(VERSION 3.8)
project(template_pkg_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

### NOTE if you want to add python libs add them here
ament_python_install_package(python_lib)
### NOTE

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


### NOTE add Cpp nodes
add_executable(node src/node.cpp)
target_include_directories(node PUBLIC
### NOTE you can add additional include dirs for Cpp here
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
### NOTE
  $<INSTALL_INTERFACE:include>)
target_compile_features(node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

### NOTE add dependencies to the cpp target
ament_target_dependencies(node
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
)
### NOTE


install(TARGETS node
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
