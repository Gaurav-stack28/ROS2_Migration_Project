# Velodyne input library

add_library(velodyne_input SHARED
  input.cc
)


set_target_properties(velodyne_input PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)


target_include_directories(velodyne_input
  PRIVATE
  ${rclcpp_INCLUDE_DIRS}
)


target_link_libraries(velodyne_input
  ${rclcpp_LIBRARIES}
  ${libpcap_LIBRARIES}
)


install(TARGETS velodyne_input
  LIBRARY DESTINATION lib
)