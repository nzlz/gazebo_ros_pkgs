find_package(ament_cmake_gtest REQUIRED)
find_package(cv_bridge REQUIRED)

option(ENABLE_DISPLAY_TESTS "Enable the building of tests that requires a display" OFF)

# Worlds
file(GLOB worlds RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "worlds/*.world")
foreach(world ${worlds})
  # Use configure_file so this is rerun each time make is invoked (as opposed to just cmake)
  configure_file(${world} ${world} COPYONLY)
endforeach()

# Tests
set(tests
  test_gazebo_ros_diff_drive
  test_gazebo_ros_force
  test_gazebo_ros_imu_sensor
  test_gazebo_ros_joint_state_publisher
  test_gazebo_ros_p3d
  test_gazebo_ros_ray_sensor
)

if(ENABLE_DISPLAY_TESTS)
set(tests ${tests}
  test_gazebo_ros_camera
  # TODO(louise) Test hangs on teardown while destroying 2nd node
  # test_gazebo_ros_camera_distortion
  test_gazebo_ros_camera_triggered
)
endif()

foreach(test ${tests})
  ament_add_gtest(${test}
    ${test}.cpp
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}
    # Long timeout because has to run gazebo several times
    TIMEOUT
      120
  )
  target_link_libraries(${test}
    gazebo_test_fixture
  )
  ament_target_dependencies(${test}
    "cv_bridge"
    "gazebo_dev"
    "gazebo_ros"
    "geometry_msgs"
    "image_transport"
    "nav_msgs"
    "rclcpp"
    "sensor_msgs"
  )
endforeach()

