cmake_minimum_required(VERSION 3.5)
project(rexrov2_description)

find_package(ament_cmake REQUIRED)

# Install share files
install(DIRECTORY launch urdf meshes robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
