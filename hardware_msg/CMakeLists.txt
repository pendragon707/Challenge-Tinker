cmake_minimum_required(VERSION 3.8)
project(hardware_msg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "src/msg/BoardParameters.msg"
  "src/msg/ImuParameters.msg" 
  "src/msg/MotorParameters.msg"  
  "src/msg/MotorsStates.msg"
  "src/msg/Imu.msg"              
  "src/msg/MotorData.msg"      
  "src/msg/MotorsCommands.msg"

  DEPENDENCIES std_msgs
)
