cmake_minimum_required(VERSION 3.11)
project(rosbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find packages
set(DEPENDENCIES
    ament_cmake
    nav2_msgs
    rclcpp)

foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Executables
add_executable(map_autosaver_node src/map_autosaver_main.cpp
                                  src/map_autosaver_node.cpp)
list(APPEND exec_nodes map_autosaver_node)

foreach(node ${exec_nodes})
  ament_target_dependencies(${node} ${DEPENDENCIES})
endforeach()

install(TARGETS ${exec_nodes} DESTINATION lib/${PROJECT_NAME})

# Rest of the files
install(DIRECTORY config launch layout DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)

# Export
ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

ament_package()
