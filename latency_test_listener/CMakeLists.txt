cmake_minimum_required(VERSION 3.5)
project(latency_test_listener)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # If you're using std_msgs for message passing
find_package(custom_msg REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#Create the executable
add_executable(latency_test_listener
    src/listener.cpp
    src/listener_main.cpp
    src/listener_TCP.cpp
    src/listener_UDP.cpp
    src/listener_RAW.cpp
)

target_link_libraries(latency_test_listener yaml-cpp)

ament_target_dependencies(
  latency_test_listener
  rclcpp
  std_msgs  # Include std_msgs if you're using it for your messages
  custom_msg
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the target
install(TARGETS
  latency_test_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()