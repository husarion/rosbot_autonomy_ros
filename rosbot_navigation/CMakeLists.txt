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
    nav_msgs
    rclcpp
    std_srvs)

foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Executables
add_executable(map_autosaver_node src/map_autosaver_main.cpp
                                  src/map_autosaver_node.cpp)
list(APPEND exec_nodes map_autosaver_node)

add_executable(map_eraser_node src/map_eraser_main.cpp
                               src/map_eraser_node.cpp)
list(APPEND exec_nodes map_eraser_node)

foreach(node ${exec_nodes})
  ament_target_dependencies(${node} ${DEPENDENCIES})
endforeach()

install(TARGETS ${exec_nodes} DESTINATION lib/${PROJECT_NAME})

# Rest of the files
install(PROGRAMS scripts/autonomy_preflight DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch layout DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_eraser test/test_map_eraser.cpp
                  src/map_eraser_node.cpp)
  ament_target_dependencies(test_map_eraser ${DEPENDENCIES})
endif()

# Export
ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

ament_package()
