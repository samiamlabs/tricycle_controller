cmake_minimum_required(VERSION 2.8.3)
project(tricycle_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    controller_interface
    nav_msgs
    realtime_tools
    tf
    urdf_vehicle_kinematic)



find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tricycle_controller.cpp src/speed_limiter.cpp src/wheel_kinematics.cpp )
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
  find_package(catkin REQUIRED COMPONENTS roscpp rostest std_srvs controller_manager tf)

  add_executable(example_driver_node test/src/example_driver_node.cpp)
  target_link_libraries(example_driver_node ${catkin_LIBRARIES})

  add_dependencies(tests example_driver_node)

endif()
