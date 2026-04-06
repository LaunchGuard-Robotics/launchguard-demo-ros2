cmake_minimum_required(VERSION 3.8)
project(demo_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/mover.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_velocity_math test/test_velocity_math.cpp)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  add_ros_isolated_launch_test(test/test_topic_published.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/test_robot_moves.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/test_speed_threshold.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/test_sensor_sanity.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/test_multi_node_comm.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/test_graceful_shutdown.py TIMEOUT 300)
endif()

ament_package()
