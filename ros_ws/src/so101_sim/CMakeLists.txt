cmake_minimum_required(VERSION 3.8)
project(so101_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  config
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  src
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  src/so101_sim/sim_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config
install(FILES
  config/so101.rviz
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Gazebo world
install(FILES
  worlds/so101_world.world
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install URDF
install(FILES
  urdf/so101_dual.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install controller config
install(FILES
  config/controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Install MoveIt config
install(FILES
  config/so101.srdf
  config/kinematics.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 