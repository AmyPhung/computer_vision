cmake_minimum_required(VERSION 3.0.2)
project(computer_vision)

find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES computer_vision
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_reconstruction src/image_reconstruction/image_reconstruction.cpp)
target_link_libraries(image_reconstruction ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})