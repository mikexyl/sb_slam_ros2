cmake_minimum_required(VERSION 3.8)
project(sb_slam_ros2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_contract
    test/test_launch_contract.py)
endif()

ament_package()
