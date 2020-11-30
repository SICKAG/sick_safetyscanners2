cmake_minimum_required(VERSION 3.5)
project(sick_safetyscanners2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sick_safetyscanners_base REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  sick_safetyscanners_base)

add_executable(sick_safetyscanners2_node 
  src/sick_safetyscanners2_node.cpp
  src/SickSafetyscannersRos2.cpp)

target_link_libraries(sick_safetyscanners2_node 
  sick_safetyscanners_base::sick_safetyscanners_base)

#TODO set dependencies here by name
ament_target_dependencies(sick_safetyscanners2_node ${dependencies})

target_include_directories(sick_safetyscanners2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#TODO new install directory for launch/ directories
#install(DIRECTORY
#launch
#DESTINATION share/${PROJECT_NAME}/
#)

install(TARGETS sick_safetyscanners2_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#TODO lookup what this does
ament_export_dependencies(${dependencies})

ament_package()
