cmake_minimum_required(VERSION 3.10)
project(mevion)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY
  models
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bin/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
