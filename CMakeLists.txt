cmake_minimum_required(VERSION 3.8)
project(gazebo_model_attachment_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(Boost REQUIRED)

# Locate msg and srv files
set(srv_files "srv/Attach.srv" "srv/Detach.srv")

#
# Install python module and scripts
#
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

include_directories(include)

# Generate msg and srv interfaces
rosidl_generate_interfaces(${PROJECT_NAME} ${srv_files})
ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}_lib SHARED src/${PROJECT_NAME}.cpp)

target_include_directories(
    ${PROJECT_NAME}_lib PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp gazebo_msgs gazebo_ros std_msgs gazebo_dev)
ament_export_dependencies(gazebo_dev rclcpp gazebo_msgs gazebo_ros std_msgs)

# Include interfaces generated in this package
rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_targets(ModelAttachment HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

install(DIRECTORY include DESTINATION include)

install(
    TARGETS ${PROJECT_NAME}_lib
    EXPORT ModelAttachment
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include
)

#
# Add ros tests
#
enable_testing()
find_file(NOSETESTS_EXECUTABLE nosetests)
add_test(nosetests "${NOSETESTS_EXECUTABLE}" -v --with-xunit)

install(DIRECTORY test DESTINATION share/${PROJECT_NAME})

find_package(launch_testing_ament_cmake)
add_launch_test(test/test_plugin.test.py)

ament_package()
