cmake_minimum_required(VERSION 3.8)
project(
  hector_stability_metrics
  VERSION 0.8
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ##############################################################################
# Library  ##
# ##############################################################################

# Header-only interface library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
  ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

# ##############################################################################
# Install   ##
# ##############################################################################

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# ##############################################################################
# Testing ##
# ##############################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_stability_measures test/test_stability_measures.cpp)
  if(TARGET test_stability_measures)
    target_link_libraries(test_stability_measures ${PROJECT_NAME})
    target_include_directories(test_stability_measures
                               PRIVATE ${EIGEN3_INCLUDE_DIRS})
  endif()

  ament_add_gtest(test_tipping test/test_tipping.cpp)
  if(TARGET test_tipping)
    target_link_libraries(test_tipping ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_headers test/test_headers.cpp)
  if(TARGET test_headers)
    target_link_libraries(test_headers ${PROJECT_NAME})
    target_include_directories(test_headers PRIVATE ${EIGEN3_INCLUDE_DIRS})
  endif()

  ament_add_gtest(test_convex_hull test/test_convex_hull.cpp)
  if(TARGET test_convex_hull)
    target_link_libraries(test_convex_hull ${PROJECT_NAME})
    target_include_directories(test_convex_hull PRIVATE ${EIGEN3_INCLUDE_DIRS})
  endif()

  # Optional coverage support
  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    include(CodeCoverage)
    append_coverage_compiler_flags()
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/build/*")
    add_code_coverage(NAME ${PROJECT_NAME}_coverage EXECUTABLE ctest)
  endif()
endif()

# ##############################################################################
# Export    ##
# ##############################################################################

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
