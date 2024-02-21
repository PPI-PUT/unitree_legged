cmake_minimum_required(VERSION 3.14)
project(unitree_a1_driver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

add_compile_options(-Wall -Wextra -pedantic -Werror)
add_compile_options(-Wno-missing-field-initializers)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
  ${EIGEN3_INCLUDE_DIRS}
)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/
  DESTINATION lib
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(UNITREE_A1_DRIVER_LIB_SRC
  src/unitree_a1_driver.cpp
  src/unitree_a1_driver_converter.cpp
)

set(UNITREE_A1_DRIVER_LIB_HEADERS
  include/unitree_a1_driver/unitree_a1_driver.hpp
  include/unitree_a1_driver/unitree_a1_driver_converter.hpp
  include/unitree_a1_driver/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${UNITREE_A1_DRIVER_LIB_SRC}
  ${UNITREE_A1_DRIVER_LIB_HEADERS}
)
target_link_libraries(${PROJECT_NAME} ${EXTRA_LIBS})
set(UNITREE_A1_DRIVER_NODE_SRC
  src/unitree_a1_driver_node.cpp
)

set(UNITREE_A1_DRIVER_NODE_HEADERS
  include/unitree_a1_driver/unitree_a1_driver_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${UNITREE_A1_DRIVER_NODE_SRC}
  ${UNITREE_A1_DRIVER_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "unitree_a1_driver::UnitreeLeggedNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
