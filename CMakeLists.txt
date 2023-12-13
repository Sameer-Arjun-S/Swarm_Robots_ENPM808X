cmake_minimum_required(VERSION 3.8)
project(hydra)

# Set compiler options
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(include)

# Define the library
add_library(library SHARED
  src/publisher.cpp
  src/master.cpp
  src/custom_trajectory.cpp
)
target_include_directories(library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
ament_target_dependencies(library
  tf2
  tf2_ros
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Link dependencies (if any)
# target_link_libraries(library <other_dependencies_if_any>)

# Install the library
install(TARGETS library
  DESTINATION lib/${PROJECT_NAME}
)

# Install other project directories and files
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

# Coverage and testing configuration (if applicable)
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)
  set(ARG_CONFIG_FILE "${PROJECT_SOURCE_DIR}/uncrustify_google_cpp_allman.cfg")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TESTFILES
    test/main.cpp
    test/test.cpp
    test/publisher_test.cpp
    src/custom_trajectory.cpp
    include/custom_trajectory.hpp
  )
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME}
    tf2
    tf2_ros
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
  )

  install(TARGETS test_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
  endif()

if(WANT_COVERAGE)
  # unit test coverage:
  setup_target_for_coverage_lcov ( # use lcov to generate coverage report
  NAME test_coverage          # target name
  EXECUTABLE ctest || true    # what to run insdie the build directory?
  #                             #   '|| true' --> continue coverage even if ctest failed
  EXECUTABLE ctest     # what to run insdie the build directory?
  EXCLUDE
  )

endif()

# create targets for building code coverage reports


# # Install scripts
# install(PROGRAMS scripts/generate_coverage_report.bash
#   DESTINATION lib/${PROJECT_NAME}
# )

# Finalize the ament package
ament_package()
