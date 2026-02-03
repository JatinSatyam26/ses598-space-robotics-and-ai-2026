cmake_minimum_required(VERSION 3.8)
project(first_order_boustrophedon_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerformanceMetrics.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python package
install(DIRECTORY
  first_order_boustrophedon_navigator
  DESTINATION lib/python3.12/site-packages/
)

# Install Python executables (with .py)
install(PROGRAMS
  first_order_boustrophedon_navigator/boustrophedon_controller.py
  first_order_boustrophedon_navigator/boustrophedon_optimizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create symlinks without .py extension
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink 
  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/boustrophedon_controller.py
  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/boustrophedon_controller)")

ament_package()
