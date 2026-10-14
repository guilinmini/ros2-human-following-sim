cmake_minimum_required(VERSION 3.8)
project(care_robot_description)

find_package(ament_cmake REQUIRED)

# Keep the large upstream Collada body compressed in the source tree.
# Unpack at configure time so installed package:// URIs work in Gazebo and RViz.
set(SU7_MESH_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/meshes/su7ultra")
set(SU7_MESH_BUILD "${CMAKE_CURRENT_BINARY_DIR}/meshes/su7ultra")
file(MAKE_DIRECTORY "${SU7_MESH_BUILD}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${SU7_MESH_SOURCE}/car.zip")
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E tar xf "${SU7_MESH_SOURCE}/car.zip"
  WORKING_DIRECTORY "${SU7_MESH_BUILD}"
  RESULT_VARIABLE SU7_EXTRACT_RESULT
)
if(NOT SU7_EXTRACT_RESULT EQUAL 0)
  message(FATAL_ERROR "Could not unpack the SU7 Ultra body mesh")
endif()

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes
  PATTERN "*.zip" EXCLUDE)
install(FILES "${SU7_MESH_BUILD}/car.dae"
  DESTINATION share/${PROJECT_NAME}/meshes/su7ultra)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
