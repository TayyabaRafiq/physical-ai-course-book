cmake_minimum_required(VERSION 3.8)
project(vla_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Declare action files
set(action_files
  "action/PickObject.action"
  "action/PlaceObject.action"
  "action/NavigateToPoint.action"
  "action/InspectObject.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES
    geometry_msgs
    std_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()