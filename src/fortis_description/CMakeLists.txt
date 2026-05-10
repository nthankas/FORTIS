cmake_minimum_required(VERSION 3.8)
project(fortis_description)

find_package(ament_cmake REQUIRED)

# Install the placeholder asset directories so they are discoverable
# under the package share once the actual content lands. Each directory
# carries a placeholder marker so it survives the install step before
# any real files exist.
install(
  DIRECTORY
    urdf
    meshes
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
