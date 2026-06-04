cmake_minimum_required(VERSION 3.8)
project(aimee_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Message files
set(msg_files
  "msg/Intent.msg"
  "msg/Transcription.msg"
  "msg/RobotState.msg"
  "msg/FaceDetection.msg"
  "msg/TrackingCommand.msg"
  "msg/WakeWordDetection.msg"
  "msg/MotorAction.msg"
  "msg/CameraAction.msg"
  "msg/LEDAction.msg"
  "msg/CloudIntent.msg"
  # Vision/Manipulation messages
  "msg/ObjectDetection.msg"
  "msg/GraspPose.msg"
  "msg/ArmCommand.msg"
  "msg/MapInfo.msg"
)

# Service files (add as needed)
set(srv_files
  "srv/CaptureSnapshot.srv"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/ListMaps.srv"
  "srv/DeleteMap.srv"
  "srv/ImportMap.srv"
  "srv/ExportMap.srv"
)

# Action files
set(action_files
  "action/LLMGenerate.action"
  "action/ExecuteSkill.action"
  "action/PickPlace.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
