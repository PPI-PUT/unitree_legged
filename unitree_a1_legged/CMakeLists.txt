cmake_minimum_required(VERSION 3.8)
project(unitree_a1_legged)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-Wno-missing-field-initializers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(unitree_a1_legged_msgs REQUIRED)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

include_directories(include
                    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
                    )

install(
        DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/
        DESTINATION lib
        )
link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(dependencies
    unitree_a1_legged_msgs
    rclcpp
)

add_library(unitree_legged
    src/unitree_legged.cpp
    src/unitree_legged_converter.cpp
    src/unitree_legged_node.cpp
)
target_link_libraries(unitree_legged ${EXTRA_LIBS})
# ament_target_dependencies(unitree_legged ${dependencies} rclcpp_components)
ament_target_dependencies(unitree_legged ${dependencies})

# rclcpp_components_register_nodes(
#   unitree_legged
#   PLUGIN "unitree_a1_legged::UnitreeLeggedNode"
#   EXECUTABLE ${PROJECT_NAME}_node_exe)

add_executable(example_position src/example_position.cpp)
ament_target_dependencies(example_position rclcpp ${dependencies})

add_executable(udp src/udp.cpp)
target_link_libraries(udp unitree_legged)

install(TARGETS
  unitree_legged
  udp
  example_position
  DESTINATION lib/${PROJECT_NAME}
  )
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_libraries(unitree_legged)
ament_export_dependencies(${dependencies})
ament_package()