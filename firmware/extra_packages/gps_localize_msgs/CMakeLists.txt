cmake_minimum_required(VERSION 3.8)
project(gps_localize_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# No dependencies on other message packages on purpose: every field is a
# float32, so micro-ROS can generate type support for this with nothing extra
# pulled into the firmware image.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Telemetry.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
