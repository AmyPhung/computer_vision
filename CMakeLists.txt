cmake_minimum_required(VERSION 3.0.2)
project(computer_vision)

find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
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

add_executable(visual_odometry_node src/visual_odometry.cpp)
target_link_libraries(visual_odometry_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})