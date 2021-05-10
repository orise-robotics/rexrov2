cmake_minimum_required(VERSION 3.5)
project(rexrov2_description)

find_package(ament_cmake REQUIRED)

# Auto lint
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()
  add_launch_test(test/upload.test.py)
endif()

# Install share files
install(DIRECTORY launch urdf meshes robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
