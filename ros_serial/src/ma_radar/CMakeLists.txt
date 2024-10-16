cmake_minimum_required(VERSION 3.0.2)
project(ma_radar)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  pcl_ros
  sensor_msgs
  std_msgs
  tf
  message_generation
)
add_message_files(
  FILES
  Point2f.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


find_package(serial REQUIRED)
find_package(OpenCV)
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(OpenVINO)
message(${ZED_INCLUDE_DIRS})
message("${ZED_LIBRARIES}")


include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/opt/intel/openvino_2023/runtime/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ma_radar
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ma src/ma_radar_serial.cpp src/crc.cpp )
add_executable(MA  src/main5.cpp)

target_link_libraries(ma
  PRIVATE
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  ${OpenCV_LIBS}
  ${ZED_LIBRARIES}
  openvino::runtime
)

target_link_libraries(MA
  PRIVATE
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  ${OpenCV_LIBS}
  ${ZED_LIBRARIES}
  openvino::runtime
)

