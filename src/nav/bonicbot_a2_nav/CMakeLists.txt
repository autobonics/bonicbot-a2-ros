cmake_minimum_required(VERSION 3.8)
project(bonicbot_a2_nav)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/vision_pipeline.py
  scripts/object_follower.py
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
