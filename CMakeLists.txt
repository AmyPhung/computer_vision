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

add_executable(visual_odometry src/visual_odometry/visual_odometry.cpp)
add_executable(feature_detection src/feature_detection/feature_detection.cpp)

target_link_libraries(visual_odometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(feature_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})