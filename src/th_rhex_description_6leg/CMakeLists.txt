cmake_minimum_required(VERSION 3.8)
project(th_rhex_description_6leg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz textures urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/validate_urdf.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
