cmake_minimum_required(VERSION 3.8)
project(vision)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
LIST(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/CMake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

# # Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  vision.srv
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
scripts/server.py scripts/client.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(vision_client src/client.cpp)
target_link_libraries(vision_client ${catkin_LIBRARIES})
add_dependencies(vision_client ${vision_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS vision_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
