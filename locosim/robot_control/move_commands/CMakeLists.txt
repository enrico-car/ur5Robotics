cmake_minimum_required(VERSION 2.8.3)
project(move_commands)

# # Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs vision gazebo_ros_link_attacher)
find_package(Eigen3 3.3 REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(JSONCPP jsoncpp)

# # Declare a catkin package
catkin_package(
  LIBRARIES move_commands
)



# # Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${JSONCPP_INCLUDE_DIRS})

add_executable(assignment_123 src/assignment_123.cpp)
add_executable(assignment_4 src/assignment_4.cpp)
add_library(JointStatePublisher src/joint_state_publisher.cpp)
add_library(Kinematic src/kinematic.cpp)
add_library(Costants src/constants.cpp)
add_library(Block src/block.cpp)
add_library(Algebra src/algebra.cpp)

target_link_libraries(assignment_123 Costants JointStatePublisher Kinematic Block Algebra ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
target_link_libraries(assignment_4 Costants JointStatePublisher Kinematic Block Algebra ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})

install(TARGETS assignment_123
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS assignment_4
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS JointStatePublisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS Kinematic
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS Costants
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS Block
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS Algebra
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)