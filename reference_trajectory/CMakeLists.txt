cmake_minimum_required(VERSION 3.0.2)
project(reference_trajectory)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  UavBsplineTrajectory.msg
  UavFlatTrajectory.msg
  UavFlatTrajectoryPoint.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reference_trajectory_nmpc
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp rospy std_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(reference_trajectory_nmpc
  src/nmpc_reference_trajectory.cpp
)
add_dependencies(reference_trajectory_nmpc ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                      ${catkin_EXPORTED_TARGETS})
target_link_libraries(reference_trajectory_nmpc
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS reference_trajectory_nmpc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/publish_goal.py
  scripts/publish_uav_reference_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(reference_trajectory_unit_test
    test/reference_trajectory_unit_test.cpp
  )
  target_link_libraries(reference_trajectory_unit_test
    reference_trajectory_nmpc
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(reference_trajectory_smoke_test
    test/reference_trajectory_smoke_test.cpp
  )
  target_link_libraries(reference_trajectory_smoke_test
    reference_trajectory_nmpc
    ${catkin_LIBRARIES}
  )
endif()
