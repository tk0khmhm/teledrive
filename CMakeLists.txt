cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          S: w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
find_package(Eigen)
include_directories(/usr/include/eigen3)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#generate_server
rosbuild_add_executable(generate_server src/generate_server/generate_server.cpp)
target_link_libraries(generate_server trajectory_generation visualize boundary_state local_plan)

#motion_player
rosbuild_add_executable(motion_player src/motion_player/motion_player.cpp)

#trajectory_manager
rosbuild_add_executable(trajectory_manager src/generate_server/trajectory_manager.cpp)
target_link_libraries(trajectory_manager visualize)

#collision checker
rosbuild_add_executable(collision_checker src/collision_checker/collision_checker.cpp)
target_link_libraries(collision_checker visualize local_plan)

#table-look-up_creator
rosbuild_add_executable(look-up-table_creator src/generate_server/look-up-table_creator.cpp)
target_link_libraries(look-up-table_creator trajectory_generation)

rosbuild_add_executable(control_infant src/motion_player/control_infant.cpp)


rosbuild_add_executable(topgun src/joy_trans/topgun.cpp)
rosbuild_add_executable(radicon src/joy_trans/radicon.cpp)
