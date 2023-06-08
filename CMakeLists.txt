cmake_minimum_required(VERSION 3.0.2)
project(ros-global-planner-plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wno-error=shadow)

## Find packages
set(PKG_DEPS
  roscpp
  rospy
  std_msgs
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  navfn
  nav_msgs
  pluginlib
  roscpp
  tf2_ros
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  image_geometry
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  control_msgs
  actionlib
  cv_bridge
  image_transport
  compressed_depth_image_transport
  image_geometry
  geometric_shapes
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_msgs
  nav_core 
  sensor_msgs
  costmap_2d
  dynamic_reconfigure
  navfn
  nav_msgs
  pluginlib
)

find_package(OpenCV REQUIRED 
    COMPONENTS
    opencv_calib3d
    opencv_features2d
    opencv_core
    opencv_highgui
    opencv_imgproc)

## Find specific system libraries
find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate services in the 'msg' folder

#add_message_files(
#  FILES
 
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES

# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   geometry_msgs 
#   actionlib
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    ${PKG_DEPS}
  DEPENDS
    #system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})


####################
##  Class config  ##
####################

add_executable(measure_time src/measure_time.cpp)

add_dependencies(measure_time ${catkin_EXPORTED_TARGETS})

target_link_libraries(measure_time
${catkin_LIBRARIES}
${PROJECT_NAME}
)



add_library(${PROJECT_NAME}
   src/roboticslab_global_planner.cpp
   src/measure_time.cpp
)



target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

##############
##  Plugin  ##
##############

add_library(roboticslab_global_planner_lib src/roboticslab_global_planner.cpp)
target_link_libraries(roboticslab_global_planner_lib
  ${OpenCV_LIBRARIES}
)



######################
##  Scripts config  ##
######################

# global planner
#add_executable(roboticslab_global_planner_node src/roboticslab_global_planner_node.cpp)

#add_dependencies(roboticslab_global_planner_node ${catkin_EXPORTED_TARGETS})

#target_link_libraries(roboticslab_global_planner_node
#${catkin_LIBRARIES}
#${PROJECT_NAME}
#)


#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
