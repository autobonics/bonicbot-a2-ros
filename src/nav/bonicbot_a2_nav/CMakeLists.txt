cmake_minimum_required(VERSION 3.8)
project(bonicbot_a2_nav)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/vision_pipeline.py
  scripts/object_follower.py
  # Docking addon. Installed unconditionally — CMake cannot know a robot's
  # fitment, and an installed script that nothing launches costs nothing.
  # docking.launch.py refers to it as `dock_pose_publisher.py`, extension
  # INCLUDED, which is how install(PROGRAMS) names it here.
  scripts/dock_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
