cmake_minimum_required(VERSION 3.8)
project(sub_guidage_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(guidage_base src/guidage_base.cpp)
ament_target_dependencies(guidage_base rclcpp std_msgs geometry_msgs)

add_executable(guidage_simple src/guidage_simple.cpp)
ament_target_dependencies(guidage_simple rclcpp std_msgs geometry_msgs)

add_executable(guidage_sonar1 src/guidage_sonar1.cpp)
ament_target_dependencies(guidage_sonar1 rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(guidage_sonar2 src/guidage_sonar2.cpp)
ament_target_dependencies(guidage_sonar2 rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(guidage_sonar3 src/guidage_sonar3.cpp)
ament_target_dependencies(guidage_sonar3 rclcpp std_msgs geometry_msgs nav_msgs)

install(TARGETS
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS
  guidage_base
  guidage_simple
  guidage_sonar1
  guidage_sonar2
  guidage_sonar3
  DESTINATION lib/${PROJECT_NAME})

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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install resource files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
