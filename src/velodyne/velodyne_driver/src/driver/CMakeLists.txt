# -----------------------------
# Normal ROS2 executable
# -----------------------------

add_executable(velodyne_node
  velodyne_node.cc
  driver.cc
)

target_link_libraries(velodyne_node
  velodyne_input
  ${rclcpp_LIBRARIES}
  ${diagnostic_updater_LIBRARIES}
  ${velodyne_msgs_LIBRARIES}
  ${libpcap_LIBRARIES}
)


# -----------------------------
# ROS2 component
# replacement for ROS1 nodelet
# -----------------------------

add_library(velodyne_driver_component SHARED
  component.cc
  driver.cc
)


set_target_properties(
  velodyne_driver_component
  PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)


target_link_libraries(velodyne_driver_component
  velodyne_input
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  ${diagnostic_updater_LIBRARIES}
  ${velodyne_msgs_LIBRARIES}
  ${libpcap_LIBRARIES}
)


rclcpp_components_register_nodes(
  velodyne_driver_component
  "velodyne_driver::VelodyneDriver"
)


# -----------------------------
# Install
# -----------------------------

install(TARGETS velodyne_node
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS velodyne_driver_component
  LIBRARY DESTINATION lib
)