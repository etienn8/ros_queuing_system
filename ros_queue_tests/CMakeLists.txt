cmake_minimum_required(VERSION 3.0.2)
project(ros_queue_tests)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ros_queue_msgs
  roscpp
  rosparam_utils
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   ros_queue_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros_queue_tests
  CATKIN_DEPENDS ros_queue_msgs roscpp queue_server queue_controller
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(inversed_cumuluative_distributions_lib
  src/inversed_cumulative_distribution.cpp
  src/inverted_bernoulli.cpp
  src/inverted_poisson.cpp
  src/distribution_sample_service.cpp
  src/distribution_sample_topic_size.cpp
)

add_library(distribution_sample_server
  src/distribution_sample_server.cpp
)

add_library(prediction_server_lib
  src/prediction_server.cpp
  src/prediction_service.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(distribution_sample_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(prediction_server_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(distribution_services_node 
              src/distribution_services_node.cpp)

add_executable(transmission_vector_action_server
              src/transmission_vector_action_server.cpp)
              
add_executable(prediction_server_node
              src/prediction_server_node.cpp)

add_executable(transmission_action_receiver_node
               src/transmission_action_receiver.cpp
               src/transmission_action_receiver_node.cpp)

add_executable(penalty_3q_2s_node
               src/penalty_3q_2s_node.cpp)       

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(distribution_services_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(transmission_vector_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(transmission_action_receiver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(penalty_3q_2s_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(distribution_sample_server
  ${catkin_LIBRARIES}
  inversed_cumuluative_distributions_lib
)

target_link_libraries(distribution_services_node
  ${catkin_LIBRARIES}
  inversed_cumuluative_distributions_lib
  distribution_sample_server
)

target_link_libraries(prediction_server_lib
  ${catkin_LIBRARIES}
  inversed_cumuluative_distributions_lib
)

target_link_libraries(prediction_server_node
  ${catkin_LIBRARIES}
  prediction_server_lib
)

target_link_libraries(transmission_vector_action_server
  ${catkin_LIBRARIES}
)

target_link_libraries(transmission_action_receiver_node
  ${catkin_LIBRARIES}
)
target_link_libraries(penalty_3q_2s_node
  ${catkin_LIBRARIES}
)