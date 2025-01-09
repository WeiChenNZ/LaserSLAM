cmake_minimum_required(VERSION 3.8)
project(laser_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#

set (CMAKE_CXX_STANDARD 14)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

#set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)


find_package(PCL REQUIRED)
#find_package(GLog REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS}) 
#include_directories(${sensor_msgs_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/include")
file(GLOB SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp")
 
  
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

#add_executable(talker ${SOURCE_FILES})
#ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)
 
add_executable(listener ${SOURCE_FILES})

target_link_libraries(listener ceres)
target_link_libraries(listener glog)
#target_link_libraries(listener gflags)  
  
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs Ceres glog)


install(TARGETS
  #talker 
  listener
  DESTINATION lib/${PROJECT_NAME}) 

 
  
ament_package()



