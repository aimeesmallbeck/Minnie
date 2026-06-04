cmake_minimum_required(VERSION 3.8)
project(aimee_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_python REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Build C++ extension
pybind11_add_module(_core
  cpp/src/bindings.cpp
  cpp/src/grid_map.cpp
  cpp/src/robot_centric_grid_map.cpp
  cpp/src/scan_matcher.cpp
  cpp/src/ekf_2d.cpp
  cpp/src/pose_graph.cpp
  cpp/src/global_planner.cpp
  cpp/src/dwa_local_planner.cpp
  cpp/src/math_utils.cpp
  cpp/src/mcl_2d.cpp
  cpp/src/frontier_detector.cpp
)

target_include_directories(_core PRIVATE
  cpp/include
)

# Install extension next to the Python package so it can be imported
set(PYTHON_INSTALL_DIR "local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages")

install(TARGETS _core
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Install launch, config, and scripts
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aimee_nav_node
  scripts/go_to_waypoint.py
  scripts/save_map.py
  scripts/load_map.py
  scripts/test_motion.py
  scripts/test_point_base.py
  scripts/test_imu.py
  scripts/test_90deg_turn.py
  scripts/validate_sensors.py
  scripts/test_map_manager.py
  scripts/test_mcl_frontier.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
