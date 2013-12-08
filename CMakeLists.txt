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
rosbuild_check_for_sse()
rosbuild_add_boost_directories()
rosbuild_genmsg()
rosbuild_gensrv()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
find_library(CGAL_LIBRARY CGAL /usr/lib)
find_library(GMP_LIBRARY gmp /usr/lib)
# find_package(GCAL REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${roscpp_INCLUDE_DIRS})
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
rosbuild_add_executable(path_scorer src/path_scorer.cpp)
target_link_libraries(path_scorer ${GAZEBO_LIBRARIES})

rosbuild_add_executable(dog_position_detector src/dog_position_detector.cpp)
rosbuild_add_executable(simulated_dog_position_detector src/simulated_dog_position_detector.cpp)
rosbuild_add_executable(get_path_server src/get_path_server.cpp)
rosbuild_add_executable(adjust_dog_position_action src/adjust_dog_position_action.cpp)
rosbuild_add_executable(move_robot_action src/move_robot_action.cpp)
rosbuild_add_executable(move_dog_away_action src/move_dog_away_action.cpp)
rosbuild_add_executable(map_broadcaster src/map_broadcaster.cpp)
rosbuild_add_executable(move_arm_to_base_position_action src/move_arm_to_base_position_action.cpp)
rosbuild_add_executable(avoid_dog src/avoid_dog.cpp)
rosbuild_add_executable(path_visualizer src/path_visualizer.cpp)
rosbuild_add_executable(leash_visualizer src/leash_visualizer.cpp)
rosbuild_add_executable(dog_visualizer src/dog_visualizer.cpp)
rosbuild_add_executable(high_arm_position_action src/high_arm_position_action)
rosbuild_add_executable(no_op_adjust_arm_position_action src/no_op_adjust_arm_position_action.cpp)
rosbuild_add_executable(focus_head_action src/focus_head_action.cpp)
rosbuild_add_compile_flags(focus_head_action -std=c++0x)

rosbuild_add_executable(path_visibility_measurer src/path_visibility_measurer.cpp)
rosbuild_add_executable(path_visibility_detector src/path_visibility_detector.cpp)
rosbuild_add_compile_flags(path_visibility_detector -frounding-math)
target_link_libraries(path_visibility_detector ${CGAL_LIBRARY} ${GMP_LIBRARY})

rosbuild_add_executable(dog_position_measurer src/dog_position_measurer.cpp)
rosbuild_add_executable(move_arm_to_clear_position_action src/move_arm_to_clear_position_action.cpp)
