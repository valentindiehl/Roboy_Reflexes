cmake_minimum_required(VERSION 2.8.3)
project(roboy_ros_trajectory)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation common_utilities flexrayusbinterface
)

## Find Qt5
find_package(Qt5 CONFIG REQUIRED COMPONENTS
        Core
        )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(roboy_ros_trajectory src/roboy_ros_trajectory.cpp src/RoboyBehaviourXmlParser.cpp)
add_dependencies(roboy_ros_trajectory ${catkin_EXPORTED_TARGETS})
qt5_use_modules(roboy_ros_trajectory Core)
target_link_libraries( roboy_ros_trajectory ${catkin_LIBRARIES} )
