cmake_minimum_required(VERSION 3.14)
project(unitree_a1_fixed_stand_server)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

add_compile_options(-Wall -Wextra -pedantic -Werror)

set(UNITREE_A1_FIXED_STAND_SERVER_LIB_SRC
  src/unitree_a1_fixed_stand_server.cpp
)

set(UNITREE_A1_FIXED_STAND_SERVER_LIB_HEADERS
  include/unitree_a1_fixed_stand_server/unitree_a1_fixed_stand_server.hpp
  include/unitree_a1_fixed_stand_server/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${UNITREE_A1_FIXED_STAND_SERVER_LIB_SRC}
  ${UNITREE_A1_FIXED_STAND_SERVER_LIB_HEADERS}
)

set(UNITREE_A1_FIXED_STAND_SERVER_NODE_SRC
  src/unitree_a1_fixed_stand_server_node.cpp
)

set(UNITREE_A1_FIXED_STAND_SERVER_NODE_HEADERS
  include/unitree_a1_fixed_stand_server/unitree_a1_fixed_stand_server_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${UNITREE_A1_FIXED_STAND_SERVER_NODE_SRC}
  ${UNITREE_A1_FIXED_STAND_SERVER_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "unitree_a1_fixed_stand_server::UnitreeFixedStandServerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
