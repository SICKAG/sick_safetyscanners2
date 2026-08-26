cmake_minimum_required(VERSION 3.5)
project(sick_safetyscanners2)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sick_safetyscanners_base REQUIRED)
find_package(sick_safetyscanners2_interfaces REQUIRED)

set(dependencies
        diagnostic_updater
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        sensor_msgs
        sick_safetyscanners_base
        sick_safetyscanners2_interfaces
)

add_library(sick_safetyscanners2 SHARED
        src/SickSafetyscanners.cpp
        src/SickSafetyscannersRos2.cpp
        src/utils/MessageCreator.cpp)
target_link_libraries(sick_safetyscanners2
        sick_safetyscanners_base::sick_safetyscanners_base
        ${Boost_LIBRARIES})
ament_target_dependencies(sick_safetyscanners2 ${dependencies})
target_include_directories(sick_safetyscanners2 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(sick_safetyscanners2_node
        src/sick_safetyscanners2_node.cpp)
target_link_libraries(sick_safetyscanners2_node
        sick_safetyscanners2)

add_library(sick_safetyscanners2_lifecycle SHARED
        src/SickSafetyscannersLifeCycle.cpp
        src/SickSafetyscanners.cpp
        src/utils/MessageCreator.cpp)
target_link_libraries(sick_safetyscanners2_lifecycle
        sick_safetyscanners_base::sick_safetyscanners_base
        ${Boost_LIBRARIES})
ament_target_dependencies(sick_safetyscanners2_lifecycle ${dependencies})
target_include_directories(sick_safetyscanners2_lifecycle PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(sick_safetyscanners2_lifecycle_node
        src/sick_safetyscanners2_lifecycle_node.cpp)
target_link_libraries(sick_safetyscanners2_lifecycle_node
        sick_safetyscanners2_lifecycle)

rclcpp_components_register_nodes(sick_safetyscanners2 "sick::SickSafetyscannersRos2")
rclcpp_components_register_nodes(sick_safetyscanners2_lifecycle "sick::SickSafetyscannersLifeCycle")

install(DIRECTORY
        launch description rviz
        DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(
  TARGETS 
  sick_safetyscanners2
  sick_safetyscanners2_lifecycle
  EXPORT  sick_safetyscanners2_target   # target for ament_export_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include  # for ament_export_include_directories
)

# Install executable
install(
  TARGETS 
  sick_safetyscanners2_node
  sick_safetyscanners2_lifecycle_node
  RUNTIME DESTINATION lib/sick_safetyscanners2  # ros2 run <pkg> <exe>
)

# install header files
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include) # this path is automatically added to the include path of the library
ament_export_targets(sick_safetyscanners2_target) # target name
ament_export_libraries(sick_safetyscanners2 sick_safetyscanners2_lifecycle)
ament_export_dependencies(${dependencies})  # transitive dependencies for other packages

ament_package()
