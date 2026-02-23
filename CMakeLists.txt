cmake_minimum_required(VERSION 3.8)
project(zlac8015d_cpp_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Headers propios
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---- libmodbus (recomendado vía pkg-config) ----
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

add_library(zlac_driver
  src/zlac8015d_driver.cpp
)

target_link_libraries(zlac_driver PUBLIC
  ${MODBUS_LIBRARIES}
)

target_compile_options(zlac_driver PUBLIC
  ${MODBUS_CFLAGS_OTHER}
)

# MACRO DEFINITION
target_compile_definitions(zlac_driver PRIVATE $<$<CONFIG:Debug>:DEBUG>)

ament_target_dependencies(zlac_driver
  rclcpp
  std_msgs
)

# ---- (Nodo ejecutable ----
# Si tienes algo como src/zlac_node.cpp, descomenta:
# add_executable(zlac_node src/zlac_node.cpp)
# target_link_libraries(zlac_node PRIVATE zlac_driver)
# ament_target_dependencies(zlac_node rclcpp std_msgs)
# install(TARGETS zlac_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)


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
