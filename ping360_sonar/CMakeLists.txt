cmake_minimum_required(VERSION 3.0.2)
project(ping360_sonar)

set(CMAKE_CXX_STANDARD 17)

###################################
## find ros or non-ros packages ##
###################################

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs 
  dynamic_reconfigure ping360_msgs
  image_transport
)

generate_dynamic_reconfigure_options(
  cfg/Sonar.cfg
)

add_subdirectory(dependency)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES ping360_sonar
 CATKIN_DEPENDS   roscpp rospy std_msgs sensor_msgs 
                  dynamic_reconfigure ping360_msgs
                  image_transport 
 # DEPENDS system_lib
)

###################
## Build Process ##
###################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(
  ${Boost_LIBRARY_DIR}
)


file(GLOB Source_Files 
  include/${PROJECT_NAME}/*.cpp
  src/ping360_node.cpp
)

add_executable(${PROJECT_NAME}_node ${Source_Files})
target_compile_options(${PROJECT_NAME}_node PRIVATE "-Wno-unused-function")
target_include_directories(${PROJECT_NAME}_node PRIVATE include dependency/ping-cpp/src)
target_link_libraries(${PROJECT_NAME}_node ping-cpp ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

#### TEST



#############
## Install ##
#############
