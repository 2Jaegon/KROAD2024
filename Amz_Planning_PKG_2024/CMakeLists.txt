cmake_minimum_required(VERSION 3.12)  # C++20을 사용하려면 최소 3.12 버전이 필요합니다.
project(amz_planning_pkg_2024)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)  # C++20으로 설정
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# mode: release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
# # mode: debug
# if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE Debug)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Eigen3
include_directories("/usr/include/eigen3")  # Eigen3 경로를 명시적으로 추가

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(nlohmann_json REQUIRED)

# Declare include directories
include_directories(
  include/${PROJECT_NAME} 
  include/${PROJECT_NAME}/data_header
  include/${PROJECT_NAME}/data_process_header  
  include/${PROJECT_NAME}/mission_header 
  include/${PROJECT_NAME}/path_planning_header
  ${EIGEN3_INCLUDE_DIR}
  ${CGAL_INCLUDE_DIRS}
)

# Add executable
add_executable(
  amz_planning_node
  src/amz_planning_node.cpp 
  src/path.cpp
  src/local.cpp
)

# Link libraries
target_link_libraries(amz_planning_node ${CGAL_LIBRARIES} pthread)

ament_target_dependencies(
  amz_planning_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nlohmann_json
)

# Install the executable
install(TARGETS
  amz_planning_node
  DESTINATION lib/${PROJECT_NAME}
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
