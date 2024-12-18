cmake_minimum_required(VERSION 3.5)
project(laser_line_extraction)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_line_msgs REQUIRED)

ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})

add_library(laser_line_extraction SHARED
  src/line.cpp
  src/line_extraction_node.cpp
  src/line_extraction_ros.cpp
  src/line_extraction.cpp
  include/laser_line_extraction/line.h 
  include/laser_line_extraction/line_extraction.h 
  include/laser_line_extraction/line_extraction_node.h 
  include/laser_line_extraction/utilities.h
)

include_directories(
  include include/
)

target_include_directories(laser_line_extraction PRIVATE include)

add_executable(laser_line_extraction_node src/line_extraction_node.cpp src/line_extraction_ros.cpp  src/line_extraction.cpp src/line.cpp) 

ament_target_dependencies(laser_line_extraction_node rclcpp sensor_msgs visualization_msgs geometry_msgs laser_line_msgs rclcpp) 
ament_target_dependencies(laser_line_extraction rclcpp sensor_msgs visualization_msgs geometry_msgs laser_line_msgs rclcpp) 

install (TARGETS laser_line_extraction_node
  DESTINATION lib/${PROJECT_NAME}
  )

#install(DIRECTORY launch config 
#  DESTINATION share/${PROJECT_NAME})

install(FILES include/laser_line_extraction/auto_dock.h 
  include/laser_line_extraction/line.h 
  include/laser_line_extraction/line_extraction.h 
  include/laser_line_extraction/line_extraction_node.h 
  include/laser_line_extraction/utilities.h
  DESTINATION include/laser_line_extraction
)
ament_package()
