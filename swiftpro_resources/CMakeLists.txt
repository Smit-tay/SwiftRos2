cmake_minimum_required(VERSION 3.8)
project(swiftpro_resources)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Actions
  "messaging/action/MoveArm.action"
  # Messages
  "messaging/msg/MoveToPosition.msg"
  # Services — existing
  "messaging/srv/MoveTo.srv"
  "messaging/srv/MoveToPosition.srv"
  "messaging/srv/Reset.srv"
  "messaging/srv/SetGripper.srv"
  "messaging/srv/SetPump.srv"
  # Services — new
  "messaging/srv/GetAnalog.srv"
  "messaging/srv/GetDigital.srv"
  "messaging/srv/GetServoAttach.srv"
  "messaging/srv/SetAcceleration.srv"
  "messaging/srv/SetBuzzer.srv"
  "messaging/srv/SetDigitalDirection.srv"
  "messaging/srv/SetDigitalOutput.srv"
  "messaging/srv/SetMode.srv"
  "messaging/srv/SetPolar.srv"
  "messaging/srv/SetServoAngle.srv"
  "messaging/srv/SetServoAttach.srv"
  "messaging/srv/SetWrist.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
