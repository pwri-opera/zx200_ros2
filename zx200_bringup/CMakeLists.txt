cmake_minimum_required(VERSION 3.8)
project(zx200_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_msgs REQUIRED)

install(DIRECTORY
  launch
  # config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_vehicle_plane_arguments test/test_vehicle_plane_arguments.py)
endif()

ament_package()
