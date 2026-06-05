cmake_minimum_required(VERSION 3.8)
project(catch_them_all_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(catch_them_all_project_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim catch_them_all_project_interfaces)

add_executable(main_turtle_controller src/main_turtle_controller.cpp)
ament_target_dependencies(main_turtle_controller rclcpp turtlesim catch_them_all_project_interfaces geometry_msgs)

install(
  TARGETS  
  turtle_spawner
  main_turtle_controller  
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
