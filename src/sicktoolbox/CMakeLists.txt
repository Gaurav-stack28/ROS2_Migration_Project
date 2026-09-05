cmake_minimum_required(VERSION 3.5)

project(sicktoolbox)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

###################################
## ROS2 configuration
###################################

ament_export_include_directories(include)

ament_export_libraries(
  SickLD
  SickLMS1xx
  SickLMS2xx
)

###########
## Build ##
###########

include_directories(include)

# Driver libraries

add_library(SickLD
  c++/drivers/ld/sickld/SickLD.cc
  c++/drivers/ld/sickld/SickLDBufferMonitor.cc
  c++/drivers/ld/sickld/SickLDMessage.cc
)

target_link_libraries(SickLD
  ${CMAKE_THREAD_LIBS_INIT}
)


add_library(SickLMS1xx
  c++/drivers/lms1xx/sicklms1xx/SickLMS1xx.cc
  c++/drivers/lms1xx/sicklms1xx/SickLMS1xxBufferMonitor.cc
  c++/drivers/lms1xx/sicklms1xx/SickLMS1xxMessage.cc
)

target_link_libraries(SickLMS1xx
  ${CMAKE_THREAD_LIBS_INIT}
)


add_library(SickLMS2xx
  c++/drivers/lms2xx/sicklms2xx/SickLMS2xx.cc
  c++/drivers/lms2xx/sicklms2xx/SickLMS2xxBufferMonitor.cc
  c++/drivers/lms2xx/sicklms2xx/SickLMS2xxMessage.cc
)

target_link_libraries(SickLMS2xx
  ${CMAKE_THREAD_LIBS_INIT}
)


############
## Examples
############

add_library(LDConfigLib
  c++/examples/ld/ld_config/src/ConfigFile.cpp
)


add_executable(ld_config
  c++/examples/ld/ld_config/src/main.cc
)

target_link_libraries(ld_config
  SickLD
  LDConfigLib
)


add_executable(ld_more_config
  c++/examples/ld/ld_more_config/src/main.cc
)

target_link_libraries(ld_more_config
  SickLD
)


add_executable(ld_multi_sector
  c++/examples/ld/ld_multi_sector/src/main.cc
)

target_link_libraries(ld_multi_sector
  SickLD
)


add_executable(ld_single_sector
  c++/examples/ld/ld_single_sector/src/main.cc
)

target_link_libraries(ld_single_sector
  SickLD
)


add_executable(lms1xx_simple_app
  c++/examples/lms1xx/lms1xx_simple_app/src/main.cc
)

target_link_libraries(lms1xx_simple_app
  SickLMS1xx
)


add_executable(lms2xx_config
  c++/examples/lms2xx/lms2xx_config/src/main.cc
)

target_link_libraries(lms2xx_config
  SickLMS2xx
)


add_executable(lms2xx_mean_values
  c++/examples/lms2xx/lms2xx_mean_values/src/main.cc
)

target_link_libraries(lms2xx_mean_values
  SickLMS2xx
)


add_executable(lms2xx_partial_scan
  c++/examples/lms2xx/lms2xx_partial_scan/src/main.cc
)

target_link_libraries(lms2xx_partial_scan
  SickLMS2xx
)


add_executable(lms2xx_real_time_indices
  c++/examples/lms2xx/lms2xx_real_time_indices/src/main.cc
)

target_link_libraries(lms2xx_real_time_indices
  SickLMS2xx
)


add_executable(lms2xx_set_variant
  c++/examples/lms2xx/lms2xx_set_variant/src/main.cc
)

target_link_libraries(lms2xx_set_variant
  SickLMS2xx
)


add_executable(lms2xx_simple_app
  c++/examples/lms2xx/lms2xx_simple_app/src/main.cc
)

target_link_libraries(lms2xx_simple_app
  SickLMS2xx
)


add_executable(lms2xx_stream_range_and_reflect
  c++/examples/lms2xx/lms2xx_stream_range_and_reflect/src/main.cc
)

target_link_libraries(lms2xx_stream_range_and_reflect
  SickLMS2xx
)


add_executable(lms2xx_subrange
  c++/examples/lms2xx/lms2xx_subrange/src/main.cc
)

target_link_libraries(lms2xx_subrange
  SickLMS2xx
)


#############
## Install ##
#############

install(
  TARGETS
    SickLD
    SickLMS1xx
    SickLMS2xx
    LDConfigLib
    ld_config
    ld_more_config
    ld_multi_sector
    ld_single_sector
    lms1xx_simple_app
    lms2xx_config
    lms2xx_mean_values
    lms2xx_partial_scan
    lms2xx_real_time_indices
    lms2xx_set_variant
    lms2xx_simple_app
    lms2xx_stream_range_and_reflect
    lms2xx_subrange

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hh"
)


ament_package()