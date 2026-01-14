cmake_minimum_required(VERSION 3.1.2)
project(uvdar_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_TESTS "Enable building tests" ON)


add_definitions(-DLLCP_LITTLE_ENDIAN)
add_definitions(-DLLCP_COMM_HEXADECIMAL=0)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  cmake_modules
  nodelet
  mrs_lib
  )

add_message_files(DIRECTORY msg FILES
  UwbRange.msg
  UwbRangeStamped.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LlcpRos
  CATKIN_DEPENDS roscpp
  )

include_directories(
  include
  mrs_llcp
  ${catkin_INCLUDE_DIRS}
  )

## | --------------------- UvdarRosDriver --------------------- |
add_library(llcp
  mrs_llcp/llcp.c
)

target_include_directories(llcp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mrs_llcp
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(UvdarRosDriver src/uvdar_ros_driver.cpp src/serial_port.cpp)
target_link_libraries(UvdarRosDriver
  llcp
  ${catkin_LIBRARIES}
  )

## | --------------------- UwbKalmanFilter --------------------- |
add_library(UwbKalmanFilterCore SHARED
  src/uwb_kalman/uwb_kalman_core.cpp 
)

target_include_directories(UwbKalmanFilterCore
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/UwbKalmanFilterCore
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_library(UwbKalmanFilterNodelet SHARED
  src/uwb_kalman/uwb_kalman_ros.cpp
)

target_link_libraries(UwbKalmanFilterNodelet
  UwbKalmanFilterCore
  ${catkin_LIBRARIES}
)

target_include_directories(UwbKalmanFilterNodelet
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)
  add_subdirectory(test)
endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  llcp
  UvdarRosDriver
  UwbKalmanFilterNodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)