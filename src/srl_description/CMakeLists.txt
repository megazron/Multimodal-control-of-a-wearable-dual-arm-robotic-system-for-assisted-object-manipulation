cmake_minimum_required(VERSION 3.8)
project(srl_description)

find_package(ament_cmake REQUIRED)

install(
  # MESHES WERE NEVER INSTALLED. package:// resolves to the INSTALL
  # tree, so every mesh reference in a urdf under this package failed
  # with "Unable to open file" while the file sat correctly in src/.
  DIRECTORY urdf launch rviz config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
