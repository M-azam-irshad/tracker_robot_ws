cmake_minimum_required(VERSION 3.8)
project(tracked_robot_gz_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(TrackAnimator SHARED src/TrackAnimator.cc)
set_property(TARGET TrackAnimator PROPERTY CXX_STANDARD 17)
target_link_libraries(TrackAnimator PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(TARGETS TrackAnimator DESTINATION lib)

ament_package()
