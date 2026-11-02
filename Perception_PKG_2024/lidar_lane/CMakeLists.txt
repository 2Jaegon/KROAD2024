cmake_minimum_required(VERSION 3.5)
project(lidar_lane)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  PCL
  pcl_conversions
  geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  OpenCV
)

# Link libraries
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Build the lidar_lane executable
add_executable(lidar_lane src/lidar_lane.cpp)
ament_target_dependencies(lidar_lane ${dependencies})
target_link_libraries(lidar_lane ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Install the executables
install(TARGETS
  lidar_lane
  DESTINATION lib/${PROJECT_NAME}
)

# Linting and testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package declaration
ament_package()
