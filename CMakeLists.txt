cmake_minimum_required(VERSION 3.8)
project(artebot_control LANGUAGES CXX C)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

set(MOTORIZER_SRC ${PROJECT_SOURCE_DIR}/contrib/motorizer)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${MOTORIZER_SRC}/Core/Inc)
include_directories(${MOTORIZER_SRC}/Core/Inc/modules)
# we do not have any STM32 controls here only fakes:
include_directories(${MOTORIZER_SRC}/tests/fakes/hal)
include_directories(${PROJECT_SOURCE_DIR}/hardware/include/artebot_control)

add_library(${PROJECT_NAME}
  SHARED
  ${MOTORIZER_SRC}/Core/Src/modules/crc8.c
  ${MOTORIZER_SRC}/Core/Src/modules/remote.c
  hardware/src/uart_comp.c
  hardware/src/serial_port.c
  hardware/src/artebot_system.cpp
  hardware/src/serial_motor.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/artebot_control>
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ARTEBOT_CONTROL_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface artebot_control.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/artebot_control
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/artebot_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/artebot_control
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_artebot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_artebot_system hardware/test/test_artebot_system.cpp)
  target_include_directories(test_artebot_system PRIVATE include)
  target_link_libraries(test_artebot_system ${PROJECT_NAME})
  ament_target_dependencies(test_artebot_system
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_targets(export_artebot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()