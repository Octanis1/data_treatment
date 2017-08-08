cmake_minimum_required(VERSION 2.8.3)
project(mavlink_conversion)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

add_executable(converter src/converter.cpp)
add_dependencies(converter sensor_msgs_generate_messages_cpp)
target_link_libraries(converter ${catkin_LIBRARIES})


