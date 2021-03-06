cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

# For action lib
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_check_for_sse()

find_package(Eigen REQUIRED)
find_package(gazebo PATHS  /opt/ros/groovy/stacks/simulator_gazebo/gazebo/gazebo/share/gazebo/cmake)

# Prefer static libraries
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a;.so")

find_package(CGAL PATHS ./CGAL-4.4 REQUIRED)
find_library(CGAL_LIBRARY CGAL ./CGAL-4.4/lib /usr/local/lib REQUIRED)
find_library(GMP_LIBRARY gmp /usr/lib REQUIRED) 
find_library(MPFR_LIBRARY mpfr /usr/lib/x86_64-linux-gnu REQUIRED)
set(Gperftools_DIR .)
find_package(Gperftools)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${roscpp_INCLUDE_DIRS})

rosbuild_genmsg()
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_boost_directories()

link_directories(${roscpp_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")

# Dog model plugin
add_library(dog_model_plugin SHARED src/dog_model_plugin.cpp)
set_target_properties(dog_model_plugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(dog_model_plugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(dog_model_plugin ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES})

install (TARGETS dog_model_plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

# Leash plugin
add_library(leash_model_plugin SHARED src/leash_model_plugin.cpp)
set_target_properties(leash_model_plugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(leash_model_plugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(leash_model_plugin ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES})
install (TARGETS leash_model_plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

rosbuild_add_executable(robot_driver src/robot_driver.cpp)

rosbuild_add_executable(total_force_measurer src/total_force_measurer.cpp)
rosbuild_add_executable(set_max_update_rate src/set_max_update_rate.cpp)

rosbuild_add_executable(leash_force_measurer src/leash_force_measurer.cpp)
rosbuild_add_executable(path_scorer src/path_scorer.cpp)
target_link_libraries(path_scorer ${GAZEBO_LIBRARIES})

rosbuild_add_executable(dog_position_detector src/dog_position_detector.cpp)
rosbuild_add_executable(simulated_dog_position_detector src/simulated_dog_position_detector.cpp)
rosbuild_add_executable(get_path_server src/get_path_server.cpp)
rosbuild_add_executable(adjust_dog_position_action src/adjust_dog_position_action.cpp)
rosbuild_add_executable(move_robot_action src/move_robot_action.cpp)
rosbuild_add_compile_flags(move_robot_action -std=c++0x)
rosbuild_add_executable(move_robot_local_planner_action src/move_robot_local_planner_action.cpp)
target_link_libraries(move_robot_local_planner_action ${roscpp_LIBRARIES})
rosbuild_add_compile_flags(move_robot_local_planner_action -std=c++0x)

rosbuild_add_executable(move_dog_away_action src/move_dog_away_action.cpp)
rosbuild_add_executable(map_broadcaster src/map_broadcaster.cpp)
rosbuild_add_executable(move_arm_to_base_position_action src/move_arm_to_base_position_action.cpp)
rosbuild_add_executable(avoid_dog src/avoid_dog.cpp)
rosbuild_add_executable(path_visualizer src/path_visualizer.cpp)
rosbuild_add_executable(leash_visualizer src/leash_visualizer.cpp)
rosbuild_add_executable(dog_visualizer src/dog_visualizer.cpp)
rosbuild_add_executable(high_arm_position_action src/high_arm_position_action.cpp)
rosbuild_add_executable(no_op_adjust_arm_position_action src/no_op_adjust_arm_position_action.cpp)
rosbuild_add_executable(focus_head_action src/focus_head_action.cpp)
rosbuild_add_compile_flags(focus_head_action -std=c++0x)

rosbuild_add_executable(path_visibility_measurer src/path_visibility_measurer.cpp)
rosbuild_add_executable(path_visibility_detector src/path_visibility_detector.cpp)
rosbuild_add_compile_flags(path_visibility_detector -frounding-math)
target_link_libraries(path_visibility_detector ${CGAL_LIBRARY} ${GMP_LIBRARY} ${MPFR_LIBRARY})

rosbuild_add_executable(dog_position_measurer src/dog_position_measurer.cpp)
rosbuild_add_executable(zero_height_depth_broadcaster src/zero_height_depth_broadcaster.cpp)
rosbuild_add_executable(point_arm_camera_action src/point_arm_camera_action.cpp)
rosbuild_add_executable(robot_path_scorer src/robot_path_scorer.cpp)
rosbuild_add_executable(detection_image_publisher src/detection_image_publisher.cpp)
rosbuild_add_executable(control_dog_position_behavior src/control_dog_position_behavior.cpp)
rosbuild_add_executable(path_planner src/path_planner.cpp)
