cmake_minimum_required(VERSION 3.5)
project(reachy_gazebo_gripper_glue)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/reachy_gazebo_gripper_glue.cpp
)

ament_target_dependencies(${PROJECT_NAME} gazebo_ros)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



ament_export_libraries(${PROJECT_NAME})

ament_package()
