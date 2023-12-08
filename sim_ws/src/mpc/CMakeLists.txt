cmake_minimum_required(VERSION 3.5)
project(mpc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OSQP_EIGEN_LIBRARY_DIRS /usr/local/lib)
set(OSQP_EIGEN_INCLUDE_DIRS /usr/local/include/OsqpEigen)

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  tf2_ros
  OsqpEigen)

  find_package(${PACKAGE} REQUIRED)

endforeach()

find_package(PkgConfig)
pkg_search_module(CASADI casadi)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(${OSQP_EIGEN_INCLUDE_DIRS})
link_directories(${OSQP_EIGEN_LIBRARY_DIRS})

if (NOT CASADI_LIBRARIES)
  message(SEND_ERROR "Could not find casadi.pc - did you set PKG_CONFIG_PATH?")
endif()

link_directories(${CASADI_LIBRARY_DIRS})

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

foreach(DIR
  launch
  config
  )

  install(DIRECTORY ${DIR}
          DESTINATION share/${PROJECT_NAME})
endforeach()

# Create Cpp executable
add_executable(mpc_node_cpp src/mpc_node.cpp src/mpc.cpp)

target_link_libraries(mpc_node_cpp ${CASADI_LIBRARIES})
target_include_directories(mpc_node_cpp PRIVATE ${CASADI_INCLUDEDIR})
target_link_libraries(mpc_node_cpp ipopt_interface)
target_link_libraries(mpc_node_cpp ipopt)

ament_target_dependencies(mpc_node_cpp 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros
)

# Install Cpp executables
install(TARGETS
  mpc_node_cpp
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/mpc_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
