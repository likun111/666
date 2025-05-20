cmake_minimum_required(VERSION 2.8.3)
project(robot_control_mt100)
add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  yjkj_common_msgs
  yjkj_chassis_msgs
  yjkj_control_msgs
  yjkj_log
  geometry_msgs
  nav_msgs
  yjkj_location_msgs
  tf
)

catkin_package(
#  CATKIN_DEPENDS roscpp std_msgs yjkj_common_msgs yjkj_chassis_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_socketcan_new src/control_socketcan_new.cpp)
target_link_libraries(control_socketcan_new ${catkin_LIBRARIES})

add_executable(pure_pursuit_tracking_grid_new src/pure_pursuit_tracking_grid_new.cpp)
target_link_libraries(pure_pursuit_tracking_grid_new ${catkin_LIBRARIES})

add_executable(pure_pursuit_tracking_grid_odom src/pure_pursuit_tracking_grid_odom.cpp)
target_link_libraries(pure_pursuit_tracking_grid_odom ${catkin_LIBRARIES})

add_executable(pure_pursuit_tracking src/pure_pursuit_tracking.cpp)
target_link_libraries(pure_pursuit_tracking ${catkin_LIBRARIES})

add_executable(pure_pursuit_tracking_avoid_obstacle src/pure_pursuit_tracking_avoid_obstacle.cpp)
target_link_libraries(pure_pursuit_tracking_avoid_obstacle ${catkin_LIBRARIES})

install(TARGETS control_socketcan_new
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pure_pursuit_tracking_grid_new
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pure_pursuit_tracking_grid_odom
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pure_pursuit_tracking
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pure_pursuit_tracking_avoid_obstacle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
