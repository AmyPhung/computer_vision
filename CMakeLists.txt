cmake_minimum_required(VERSION 3.0.2)
project(cv_project)

find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES cv_project
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cv_project src/main.cpp)
target_link_libraries(cv_project ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})