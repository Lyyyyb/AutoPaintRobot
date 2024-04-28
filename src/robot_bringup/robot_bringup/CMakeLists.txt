cmake_minimum_required(VERSION 2.8.3)
project(robot_bringup)



find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  serial
  std_msgs
  message_generation
  nav_msgs
)

generate_dynamic_reconfigure_options(
  cfg/speedparam.cfg
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)
########
#三个cpp文件一起运行，只能存在一个main函数，相当于把多个函数放在不同的cpp文件中
########
add_executable(robot_bringup  
               src/robot_bringup.cpp
               src/robot.cpp
               src/mbot_linux_serial.cpp
	       )
##########
#两个main函数不能共存，要用不同的add_executable
##########
add_executable(joy_control src/joy_control.cpp)


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
      
target_link_libraries(robot_bringup ${catkin_LIBRARIES})
target_link_libraries(joy_control ${catkin_LIBRARIES})




