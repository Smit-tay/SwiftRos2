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
  # Services — kinematics queries (firmware-side)
  "messaging/srv/AnglesToCoord.srv"
  "messaging/srv/CoordToAngles.srv"
  "messaging/srv/IsReachable.srv"
  # Services — kinematics queries (node-side)
  "messaging/srv/ComputeFK.srv"
  "messaging/srv/ComputeIK.srv"
  # Services — motion
  "messaging/srv/MoveTo.srv"
  "messaging/srv/PauseMotion.srv"
  "messaging/srv/ResumeMotion.srv"
  "messaging/srv/MotionReset.srv"
  "messaging/srv/Reset.srv"
  "messaging/srv/SetPolar.srv"
  "messaging/srv/SetServoAngle.srv"
  "messaging/srv/SetServoAngles.srv"
  "messaging/srv/SetWrist.srv"
  # Services — end effectors
  "messaging/srv/SetGripper.srv"
  "messaging/srv/SetPump.srv"
  "messaging/srv/SetBuzzer.srv"
  # Services — configuration
  "messaging/srv/SetAcceleration.srv"
  "messaging/srv/SetMode.srv"
  # Services — servo management
  "messaging/srv/SetServoAttach.srv"
  "messaging/srv/GetServoAttach.srv"
  "messaging/srv/GetEncoderStatus.srv"
  # Services — GPIO
  "messaging/srv/SetDigitalDirection.srv"
  "messaging/srv/SetDigitalOutput.srv"
  "messaging/srv/GetDigital.srv"
  "messaging/srv/GetAnalog.srv"
  # Services — raw passthrough
  "messaging/srv/SendRaw.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
