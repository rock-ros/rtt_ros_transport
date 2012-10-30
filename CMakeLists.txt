#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
# The CMake 'target' names are identical to the first argument of the
# macro, except for orocos_typekit, where the target is fully
# controlled by generated code of 'typegen'.
#
project(ros-transport)

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
include_directories(${OROCOS-RTT_INCLUDE_DIRS})
link_directories(${OROCOS-RTT_LIBRARY_DIRS})
add_definitions(${OROCOS-RTT_DEFINITIONS})

if(NOT DEFINED ROS_FOUND)
    find_package( ROS QUIET REQUIRED COMPONENTS roslib )
endif(NOT DEFINED ROS_FOUND)

include_directories(${ROS_INCLUDE_DIRS})
link_directories(${ROS_LIBRARY_DIRS})

add_library(orocos-rtt-ros-${OROCOS_TARGET} SHARED
    ros_publish_activity.cpp)
configure_file(orocos-rtt-ros.pc.in orocos-rtt-ros-${OROCOS_TARGET}.pc
    @ONLY)

install(FILES ros_publish_activity.hpp ros_msg_transporter.hpp
    DESTINATION include/rtt/transports/ros)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/orocos-rtt-ros-${OROCOS_TARGET}.pc
    DESTINATION lib/pkgconfig)
install(TARGETS orocos-rtt-ros-${OROCOS_TARGET}
    DESTINATION lib)
