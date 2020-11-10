cmake_minimum_required(VERSION 3.0.2)
project(hector_stability_metrics)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

set (CMAKE_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3
)

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include ${EIGEN3_INCLUDE_DIRS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_Targets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  include_directories(${eigen_checks_INCLUDE_DIRS})
  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(test_stability_measures test/test_stability_measures.test test/test_stability_measures.cpp)
  target_link_libraries(test_stability_measures ${PROJECT_NAME})

endif()

# to run: catkin build --this --no-deps -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -v --catkin-make-args hector_stability_metrics_coverage
# Path to results overview will be printed in the build process
# Big thanks to the moveit people from whose docs I've obtained the information on how to get the coverage
if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/build/*")
  add_code_coverage(NAME ${PROJECT_NAME}_coverage)
endif ()
