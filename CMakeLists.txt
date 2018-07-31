cmake_minimum_required(VERSION 2.8.3)
project(tricycle_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(${PROJECT_NAME}_CATKIN_DEPS
  controller_interface
  nav_msgs
  realtime_tools
  tf
  urdf_geometry_parser
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tricycle_controller.cpp src/odometry.cpp src/speed_limiter.cpp src/wheel_kinematics.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES tricycle_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rosgraph_msgs rostest std_srvs controller_manager tf)

  add_executable(tricycle test/src/tricycle.cpp)
  target_link_libraries(tricycle ${catkin_LIBRARIES})

  add_dependencies(tests tricycle)

  add_rostest_gtest(tricycle_controller_twist_cmd_test
                  test/tricycle_controller_twist_cmd.test
                  test/src/tricycle_twist_cmd_test.cpp)

  target_link_libraries(tricycle_controller_twist_cmd_test ${catkin_LIBRARIES})

endif()
