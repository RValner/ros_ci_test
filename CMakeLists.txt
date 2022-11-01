cmake_minimum_required(VERSION 3.0.2)
project(ros_ci_test)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  roslint
)

add_service_files(
  FILES
  SetDuration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_ci_test
CATKIN_DEPENDS roscpp rostest message_runtime
#  DEPENDS system_lib
)

# Compile the tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Simple Hz test
  add_rostest(
    test/hz_test.test
  )

  # Dynamic Hz change test
  add_rostest_gtest(dynamic_duration_test test/dynamic_duration_test.test 
    src/test/dynamic_duration_test.cpp
  )

  target_link_libraries(dynamic_duration_test ${catkin_LIBRARIES})
endif()

# invoke roslint
roslint_cpp()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dummy_pub_srv
  src/dummy_pub_srv.cpp
)

add_dependencies(dummy_pub_srv
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(dummy_pub_srv
  ${catkin_LIBRARIES}
)

install(TARGETS dummy_pub_srv
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_ci_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
