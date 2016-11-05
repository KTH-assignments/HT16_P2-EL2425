  # Get the information about this package's buildtime dependencies
  find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    std_msgs
    sensor_msgs
    slip_control_communications)

  # Declare that this catkin package's runtime dependencies
  catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
  )
