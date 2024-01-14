cmake_minimum_required(VERSION 3.0.2)
project(nmpc_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin_simple REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

catkin_simple()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES nmpc_controller
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(mpc_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_nmpc/acado_qpoases_interface.cpp
  model/quadrotor_nmpc/acado_integrator.c
  model/quadrotor_nmpc/acado_solver.c
  model/quadrotor_nmpc/acado_auxiliary_functions.c)

target_include_directories(mpc_solver PUBLIC
  model/quadrotor_nmpc/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC)

cs_add_library(mpc_wrapper
  src/mpc_wrapper.cpp)

target_link_libraries(mpc_wrapper
  mpc_solver)

cs_add_library(mpc_queue
  src/mpc_queue.cpp)

cs_add_library(nonlinear_mpc
  src/nonlinear_mpc.cpp)

target_link_libraries(nonlinear_mpc 
  mpc_wrapper mpc_queue)

cs_install()
cs_export()

cs_add_executable(nonlinear_mpc_node
  src/nonlinear_mpc_node.cpp)
target_link_libraries(nonlinear_mpc_node
  nonlinear_mpc
  mpc_wrapper
  mpc_queue
  mpc_solver)

cs_add_executable(waypoint_publisher_3rd_polynomial src/waypoint_publisher_3rd_polynomial.cpp)
target_link_libraries(waypoint_publisher_3rd_polynomial ${catkin_LIBRARIES})

cs_add_executable(waypoint_publisher_5rd_polynomial src/waypoint_publisher_5rd_polynomial.cpp)
target_link_libraries(waypoint_publisher_5rd_polynomial ${catkin_LIBRARIES})

cs_install()
cs_export()