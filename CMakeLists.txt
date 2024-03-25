cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS 
  roscpp rospy sensor_msgs nav_msgs 
  velocity_profile_msgs velocity_profile_srvs 
  utils_basic 
  tf2_ros
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs 
  velocity_profile_msgs velocity_profile_srvs
  # DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/pure_pursuit.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
