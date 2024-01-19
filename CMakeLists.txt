cmake_minimum_required(VERSION 3.8)
project(gz_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

# ROS2
find_package(rclcpp REQUIRED)
#find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Messages
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# LIBRARIES
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YAML_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(GZ_ARM_EXTERNAL_LINK_LIBS ceres Eigen3::Eigen ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} ignition-gazebo6)

# gz_arm_util lib
add_library(gz_arm_util SHARED
  src/gz_arm_util.cpp
)
ament_target_dependencies(gz_arm_util
  geometry_msgs
  tf2
  visualization_msgs
)
target_link_libraries(gz_arm_util
  ceres
  Eigen3::Eigen
)

# gz_cam_node lib
add_library(aruco_detect SHARED
  src/aruco_detect.cpp
)
ament_target_dependencies(aruco_detect
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(aruco_detect
  ignition-gazebo6
  ${OpenCV_LIBS}
)

# gz_cam_sub
add_executable(gz_cam_sub src/gz_cam_sub.cpp)
target_link_libraries(gz_cam_sub
  aruco_detect
  ignition-gazebo6
)
ament_target_dependencies(aruco_detect
  rclcpp
)


# gz_arm_control
add_library(gz_arm_control SHARED
  src/gz_arm_control.cpp
)
ament_target_dependencies(gz_arm_control
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(gz_arm_control
  ceres
  gz_arm_util
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

# gz_cam_sub
add_executable(gz_arm_exec src/gz_arm_exec.cpp)
target_link_libraries(gz_arm_exec
  aruco_detect
  gz_arm_control
  ignition-gazebo6
)
ament_target_dependencies(gz_arm_exec
  rclcpp
)

install(
  TARGETS gz_arm_util aruco_detect gz_arm_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS gz_arm_exec gz_cam_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(gz_arm_util_test test/gz_arm_util_test.cpp)
  target_include_directories(gz_arm_util_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(gz_arm_util_test
    geometry_msgs
    std_msgs
  )
  target_link_libraries(gz_arm_util_test
    gz_arm_util
    ${GZ_ARM_EXTERNAL_LINK_LIBS}
  )
endif()


ament_python_install_package( ${PROJECT_NAME} )
install(
  PROGRAMS scripts/gen_aruco.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gen_aruco
)

install(
  PROGRAMS scripts/calc_inertia.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calc_inertia
)

install(
  PROGRAMS launch/gz_arm_launch.py launch/gz_sim_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
