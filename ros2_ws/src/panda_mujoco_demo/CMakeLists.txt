cmake_minimum_required(VERSION 3.8)
project(panda_mujoco_demo)

# This package ships no C++ sources: it is a configuration/launch package that
# provides the MuJoCo MJCF, the ros2_control URDF, controller configs, and the
# launch file for the Panda pick-place demo running on mujoco_ros2_control.
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    mjcf
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
