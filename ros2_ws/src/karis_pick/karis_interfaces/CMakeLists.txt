cmake_minimum_required(VERSION 3.5)
project(karis_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPose.msg"
  "msg/ObjectPoseArray.msg"
  "srv/GetObjects.srv"
  "srv/EstimatePose.srv"
  "action/PickObject.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
