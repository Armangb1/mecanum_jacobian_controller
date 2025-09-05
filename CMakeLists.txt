cmake_minimum_required(VERSION 3.8)
project(mecanum_jacobian)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  tf2
  tf2_ros
)

find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# generate parameter library
generate_parameter_library(jacobian_controller_parameters
  src/jacobian_controller.yaml
)

generate_parameter_library(inverse_jacobian_controller_parameters
  src/inverse_jacobian_controller.yaml
)

# create the controller library
add_library(
  mecanum_jacobian
  SHARED
  src/jacobian_controller.cpp
  src/inverse_jacobian_controller.cpp
)

target_compile_features(mecanum_jacobian PUBLIC cxx_std_17)
target_include_directories(mecanum_jacobian PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mecanum_jacobian>
)
target_link_libraries(mecanum_jacobian PUBLIC jacobian_controller_parameters)
target_link_libraries(mecanum_jacobian PUBLIC inverse_jacobian_controller_parameters)
ament_target_dependencies(mecanum_jacobian PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mecanum_jacobian PRIVATE "MECANUM_JACOBIAN_BUILDING_DLL")

# export controller plugins
pluginlib_export_plugin_description_file(controller_interface mecanum_jacobian.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/mecanum_jacobian
)
install(TARGETS mecanum_jacobian jacobian_controller_parameters inverse_jacobian_controller_parameters
  EXPORT export_mecanum_jacobian
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_mecanum_jacobian HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
