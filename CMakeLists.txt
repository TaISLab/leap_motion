project(leap_motion)

cmake_minimum_required(VERSION 3.5)
project(leap_motion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)

set(dependencies
  rclcpp 
  sensor_msgs
  camera_info_manager
  image_transport
)

#custom library
add_library(lib_leap STATIC IMPORTED)
set_target_properties(lib_leap PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/LeapSDK/lib/x64/libLeap.so)
ament_export_libraries(lib_leap)

include_directories(
  include
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LeapSDK/include/>"
)

ament_export_include_directories(
  include
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LeapSDK/include/>"
)

# Node that publishes raw camera images.......................................
# Be sure to have "Allow images" enabled in LeapControlPanel!

add_executable(leap_motion_camera_node 
  src/lmc_camera_node.cpp
)
target_link_libraries(leap_motion_camera_node lib_leap)

ament_target_dependencies(leap_motion_camera_node ${dependencies})

target_include_directories(leap_motion_camera_node
 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LeapSDK/include/>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

## ........................................................................

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
leap_motion_camera_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch/
 DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY config/
 DESTINATION share/${PROJECT_NAME}/config/
)

ament_package()


# ..................................................................
# TODO: additional modules/msgs that are still not ported to ROS2  .
# ..................................................................

# Arm.msg
# Bone.msg
# Finger.msg
# Gesture.msg
# Hand.msg
# Human.msg
# # For backwards compatibility with the old driver files
# leap.msg
# leapros.msg

# ## Declare a C++ library
# add_library(lmc_listener 
#   src/lmc_listener.cpp
# )
# add_dependencies(lmc_listener 
#   ${PROJECT_NAME}_gencpp
# )
# target_link_libraries(lmc_listener
#   ${catkin_LIBRARIES}
#   ${PROJECT_SOURCE_DIR}/LeapSDK/include/Leap.h
# )

# ## Declare a C++ executable
# ## With catkin_make all packages are built within a single CMake context
# ## The recommended prefix ensures that target names across packages don't collide

# # The executable node that talks to the Leap Motion controller
# add_executable( ${PROJECT_NAME}_driver_node 
#   src/lmc_driver_node.cpp
# )
# add_dependencies( ${PROJECT_NAME}_driver_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(${PROJECT_NAME}_driver_node
#   ${catkin_LIBRARIES}
#   lmc_listener
#   ${LIB_LEAP}
# )

# # Node that visualizes the data it gets from the controller
# add_executable(${PROJECT_NAME}_visualizer_node 
#   src/lmc_visualizer_node.cpp
# )

# add_dependencies(${PROJECT_NAME}_visualizer_node  
#   ${PROJECT_NAME}_gencpp
# )
# target_link_libraries(${PROJECT_NAME}_visualizer_node  
#   ${catkin_LIBRARIES}
#   ${LIB_LEAP}
# )

# # Filter node implementing a 2nd-order Butterworth lowpass filter
# add_executable(${PROJECT_NAME}_filter_node 
#   src/lmc_filter_node.cpp
# )
# add_dependencies(${PROJECT_NAME}_filter_node 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(${PROJECT_NAME}_filter_node
#   ${catkin_LIBRARIES} 
#   ${LIB_LEAP}
# )

# #########################################################
# # For backwards compatibility with the old driver files #
# #                     DEPRECATED                        #
# #########################################################

# add_executable(leap_hands 
#   src/leap_hands.cpp
# )
# target_link_libraries(leap_hands
#   ${catkin_LIBRARIES} ${LIB_LEAP}
# )

# add_executable(leap_camera src/leap_camera.cpp)
# target_link_libraries(leap_camera
#   ${catkin_LIBRARIES} ${LIB_LEAP}
# )