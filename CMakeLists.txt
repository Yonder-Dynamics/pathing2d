cmake_minimum_required(VERSION 2.8.3)
project(pathing2d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -ggdb)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_ros
  octomap_msgs
  PCL
  pcl_conversions
)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)
find_package(OpenCV 3 REQUIRED)

## Uncomment this if the package has a setup.py
# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pathing2d
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

set(SRC
  "src/Pathing2D.cpp"
  "src/util.cpp"
)

add_executable(pathing2d_node src/pathing2d_node.cpp ${SRC})
target_link_libraries(pathing2d_node ${LINK_LIBS})
