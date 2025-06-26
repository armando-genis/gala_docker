cmake_minimum_required(VERSION 3.8)
project(image_acquisition)

# use C++17 for std::filesystem
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create the image acquisition node
add_executable(image_acquisition_node src/image.cc)

ament_target_dependencies(image_acquisition_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Install the executable
install(TARGETS
  image_acquisition_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
