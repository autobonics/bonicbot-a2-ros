cmake_minimum_required(VERSION 3.8)
project(bonicbot_a2_sim)

find_package(ament_cmake REQUIRED)

# meshes/ carries the docking station's AprilTag plate and its texture, both
# committed. obstacle_world.sdf loads them by
# package://bonicbot_a2_sim/meshes/..., which only resolves because
# sim.launch.py puts this package's install prefix on GZ_SIM_RESOURCE_PATH.
install(
  DIRECTORY config launch worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
