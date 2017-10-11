cmake_minimum_required(VERSION 2.8.3)
project(data_extraction)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)


catkin_package(

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(listener src/listener.cpp)
add_dependencies(listener sensor_msgs_generate_messages_cpp)
target_link_libraries(listener ${catkin_LIBRARIES})


