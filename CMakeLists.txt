cmake_minimum_required(VERSION 2.8.3)
project(multi_contact_point_estimator)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib std_msgs nav_msgs cv_bridge pcl_conversions vigir_generic_params vigir_pluginlib vigir_footstep_planning_msgs vigir_footstep_planning_lib vigir_footstep_planning_plugins vigir_terrain_classifier)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_contactpoint_estimator
  CATKIN_DEPENDS roscpp actionlib_msgs actionlib std_msgs nav_msgs cv_bridge pcl_conversions vigir_generic_params vigir_pluginlib vigir_footstep_planning_msgs vigir_footstep_planning_lib vigir_footstep_planning_plugins vigir_terrain_classifier
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${boost_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/uneven_terrain_stand/convex_hull_stand.h
  include/${PROJECT_NAME}/uneven_terrain_stand/model_stand.h
  include/${PROJECT_NAME}/uneven_terrain_stand/terrain_model_uneven.h
    include/${PROJECT_NAME}/uneven_terrain_stand/data_points.h
	include/${PROJECT_NAME}/uneven_terrain_stand/my_tensorflow_model_run_lib.h
	include/${PROJECT_NAME}/uneven_terrain_stand/foot_form.h
	include/${PROJECT_NAME}/uneven_terrain_stand/foot_state_uneven.h
	include/${PROJECT_NAME}/uneven_terrain_stand/uneven_terrain_stand.h
	include/${PROJECT_NAME}/uneven_terrain_stand/stand_predict.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/libqhull_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/geom_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/io_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/mem_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/merge_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/poly_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/qhull_ra.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/qset_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/random_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/stat_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/user_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/mem_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/qset_r.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/Coordinates.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/functionObjects.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/PointCoordinates.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/Qhull.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullError.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullFacet.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullFacetList.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullFacetSet.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullHyperplane.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullIterator.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullLinkedList.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullPoint.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullPoints.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullPointSet.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullQh.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullRidge.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullSet.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullSets.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullStat.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullVertex.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullVertexSet.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/RboxPoints.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/RoadError.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/RoadLogEvent.h
	include/${PROJECT_NAME}/uneven_terrain_stand/src/qhulltest/RoadTest.h
)

set(SOURCES
  src/uneven_terrain_stand/convex_hull_stand.cpp
  src/uneven_terrain_stand/model_stand.cpp
  src/uneven_terrain_stand/terrain_model_uneven.cpp
  src/uneven_terrain_stand/data_points.cpp
	src/uneven_terrain_stand/foot_form.cpp
	src/uneven_terrain_stand/uneven_terrain_stand.cpp
	src/uneven_terrain_stand/stand_predict.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/global_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/stat_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/geom2_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/poly2_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/merge_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/libqhull_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/geom_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/poly_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/qset_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/mem_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/random_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/usermem_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/io_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/user_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/rboxlib_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/qset_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/mem_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhull_r/usermem_r.c
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/Coordinates.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/PointCoordinates.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/Qhull.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullFacet.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullFacetList.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullFacetSet.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullHyperplane.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullPoint.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullPointSet.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullPoints.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullQh.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullRidge.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullSet.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullStat.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullVertex.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/QhullVertexSet.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/RboxPoints.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/RoadError.cpp
	include/${PROJECT_NAME}/uneven_terrain_stand/src/libqhullcpp/RoadLogEvent.cpp
)


## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Declare a cpp executable
# add_executable(footstep_planning_plugins_node src/visualization/footstep_planning_plugins_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/uneven_terrain_stand/lib_tensorflow_model_run.so)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*~" EXCLUDE
)

install(FILES default_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vigir_footstep_planner_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
