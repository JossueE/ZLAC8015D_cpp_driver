cmake_minimum_required(VERSION 3.8)
project(zlac8015d_cpp_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs)
find_package(PkgConfig REQUIRED)

pkg_check_modules(MODBUS REQUIRED IMPORTED_TARGET libmodbus)

add_library(zlac8015d_driver
  src/zlac8015d_driver/zlac8015d_driver.cpp
)

target_include_directories(zlac8015d_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(zlac8015d_driver
  PkgConfig::MODBUS
)

add_executable(zlac_node
  src/zlac_node.cpp
)

target_link_libraries(zlac_node
  zlac8015d_driver
)

ament_target_dependencies(zlac_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_compile_definitions(zlac_node PRIVATE
  $<$<CONFIG:Debug>:DEBUG>
)

add_executable(test_driver
  src/zlac8015d_driver/test_driver.cpp
)

target_include_directories(test_driver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_driver
  zlac8015d_driver
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    zlac8015d_driver
    zlac_node
    test_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()