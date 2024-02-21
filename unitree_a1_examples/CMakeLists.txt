cmake_minimum_required(VERSION 3.14)
project(unitree_a1_examples)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

add_compile_options(-Wall -Wextra -pedantic -Werror)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../unitree_a1_driver/unitree_legged_sdk/include
)

set(UNITREE_A1_EXAMPLES_LIB_SRC
  src/unitree_a1_examples.cpp
)

set(UNITREE_A1_EXAMPLES_LIB_HEADERS
  include/unitree_a1_examples/unitree_a1_examples.hpp
  include/unitree_a1_examples/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${UNITREE_A1_EXAMPLES_LIB_SRC}
  ${UNITREE_A1_EXAMPLES_LIB_HEADERS}
)

set(UNITREE_A1_EXAMPLES_NODE_SRC
  src/unitree_a1_example_fixed_stand_node.cpp
)

set(UNITREE_A1_EXAMPLES_NODE_HEADERS
  include/unitree_a1_examples/unitree_a1_example_fixed_stand_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${UNITREE_A1_EXAMPLES_NODE_SRC}
  ${UNITREE_A1_EXAMPLES_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "unitree_a1_examples::UnitreeFixedStandNode"
  EXECUTABLE ${PROJECT_NAME}_fixed_stand_node_exe
)

set(UNITREE_A1_EXAMPLES_POSITION_NODE_SRC
  src/unitree_a1_example_position_node.cpp
)

set(UNITREE_A1_EXAMPLES_POSITION_NODE_HEADERS
  include/unitree_a1_examples/unitree_a1_example_position_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_position_control_node SHARED
  ${UNITREE_A1_EXAMPLES_POSITION_NODE_SRC}
  ${UNITREE_A1_EXAMPLES_POSITION_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_position_control_node
  PLUGIN "unitree_a1_examples::UnitreeExamplePositionNode"
  EXECUTABLE ${PROJECT_NAME}_position_control_exe
)

set(UNITREE_A1_EXAMPLES_TORQUE_NODE_SRC
  src/unitree_a1_example_torque_node.cpp
)

set(UNITREE_A1_EXAMPLES_TORQUE_NODE_HEADERS
  include/unitree_a1_examples/unitree_a1_example_torque_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_torque_control_node SHARED
  ${UNITREE_A1_EXAMPLES_TORQUE_NODE_SRC}
  ${UNITREE_A1_EXAMPLES_TORQUE_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_torque_control_node
  PLUGIN "unitree_a1_examples::UnitreeExampleTorqueNode"
  EXECUTABLE ${PROJECT_NAME}_torque_control_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
