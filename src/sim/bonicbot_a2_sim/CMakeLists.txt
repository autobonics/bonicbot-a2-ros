cmake_minimum_required(VERSION 3.8)
project(bonicbot_a2_sim)

find_package(ament_cmake REQUIRED)

# meshes/ carries the docking dock's AprilTag plate (and its generated PNG
# texture, which scripts/make_apriltag_texture.sh writes into the SOURCE tree —
# so it must be regenerated before the build that is meant to install it).
install(
  DIRECTORY config launch worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
