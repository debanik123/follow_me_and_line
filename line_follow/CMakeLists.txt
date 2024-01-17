cmake_minimum_required(VERSION 3.8)
project(line_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV 4 REQUIRED)


set(dependencies
ament_cmake
rclcpp
std_msgs
geometry_msgs
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(_include_dirs "include;/usr/include;/usr/include/opencv4")


add_executable(color_thresholding src/color_thresholding.cpp)
ament_target_dependencies(color_thresholding ${dependencies})
target_link_libraries(color_thresholding ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

add_executable(line_following src/line_following.cpp)
ament_target_dependencies(line_following ${dependencies})
target_link_libraries(line_following ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

install(TARGETS
  color_thresholding
  line_following
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
