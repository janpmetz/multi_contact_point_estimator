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
## Get External Project ##
###########

# We need the qhull source for our convex hull algorithm 
set(QHULL_SOURCE_DIR "${PROJECT_SOURCE_DIR}/qhull")
if(NOT EXISTS "${QHULL_SOURCE_DIR}/libqhull")
	set(QHULL_FILENAME "qhull-2015-src-7.2.0.tgz")
	set(QHULL_URL "http://www.qhull.org/download/${QHULL_FILENAME}")
	file(DOWNLOAD ${QHULL_URL} ${CMAKE_CURRENT_BINARY_DIR}/${QHULL_FILENAME}
	     STATUS SHOW_PROGRESS
	     EXPECTED_MD5 e6270733a826a6a7c32b796e005ec3dc
	)
	execute_process(COMMAND ${CMAKE_COMMAND} -E tar xvf ${CMAKE_CURRENT_BINARY_DIR}/${QHULL_FILENAME})
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory qhull-2015.2/src/ ${QHULL_SOURCE_DIR})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${boost_INCLUDE_DIRS} ${QHULL_SOURCE_DIR})

## Specify additional locations of header files
set(HEADERS
	include/${PROJECT_NAME}/uneven_terrain_stand/convex_hull_stand.h
	include/${PROJECT_NAME}/uneven_terrain_stand/model_stand.h
	include/${PROJECT_NAME}/uneven_terrain_stand/terrain_model_uneven.h
	include/${PROJECT_NAME}/uneven_terrain_stand/multi_contact_point_model_run_lib.h
	include/${PROJECT_NAME}/uneven_terrain_stand/foot_form.h
	include/${PROJECT_NAME}/uneven_terrain_stand/foot_state_uneven.h
	include/${PROJECT_NAME}/uneven_terrain_stand/uneven_terrain_stand.h
	include/${PROJECT_NAME}/uneven_terrain_stand/q_hull_extended.h
	
)

set(SOURCES
	src/uneven_terrain_stand/convex_hull_stand.cpp
	src/uneven_terrain_stand/model_stand.cpp
	src/uneven_terrain_stand/terrain_model_uneven.cpp
	src/uneven_terrain_stand/foot_form.cpp
	src/uneven_terrain_stand/uneven_terrain_stand.cpp
	src/uneven_terrain_stand/q_hull_extended.cpp
)

set(QHULL_SOURCE 
	${QHULL_SOURCE_DIR}/libqhull_r/libqhull_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/geom_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/io_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/mem_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/merge_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/poly_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/qhull_ra.h
	${QHULL_SOURCE_DIR}/libqhull_r/qset_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/random_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/stat_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/user_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/mem_r.h
	${QHULL_SOURCE_DIR}/libqhull_r/qset_r.h
	${QHULL_SOURCE_DIR}/libqhullcpp/Coordinates.h
	${QHULL_SOURCE_DIR}/libqhullcpp/functionObjects.h
	${QHULL_SOURCE_DIR}/libqhullcpp/PointCoordinates.h
	${QHULL_SOURCE_DIR}/libqhullcpp/Qhull.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullError.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullFacet.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullFacetList.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullFacetSet.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullHyperplane.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullIterator.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullLinkedList.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullPoint.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullPoints.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullPointSet.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullQh.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullRidge.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullSet.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullSets.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullStat.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullVertex.h
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullVertexSet.h
	${QHULL_SOURCE_DIR}/libqhullcpp/RboxPoints.h
	${QHULL_SOURCE_DIR}/libqhullcpp/RoadError.h
	${QHULL_SOURCE_DIR}/libqhullcpp/RoadLogEvent.h
	${QHULL_SOURCE_DIR}/qhulltest/RoadTest.h
	${QHULL_SOURCE_DIR}/libqhull_r/global_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/stat_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/geom2_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/poly2_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/merge_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/libqhull_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/geom_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/poly_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/qset_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/mem_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/random_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/usermem_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/io_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/user_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/rboxlib_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/qset_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/mem_r.c
	${QHULL_SOURCE_DIR}/libqhull_r/usermem_r.c
	${QHULL_SOURCE_DIR}/libqhullcpp/Coordinates.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/PointCoordinates.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/Qhull.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullFacet.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullFacetList.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullFacetSet.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullHyperplane.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullPoint.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullPointSet.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullPoints.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullQh.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullRidge.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullSet.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullStat.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullVertex.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/QhullVertexSet.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/RboxPoints.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/RoadError.cpp
	${QHULL_SOURCE_DIR}/libqhullcpp/RoadLogEvent.cpp
)


## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${QHULL_SOURCE})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Declare a cpp executable
# add_executable(footstep_planning_plugins_node src/visualization/footstep_planning_plugins_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PROJECT_SOURCE_DIR}/tensorflow/lib/lib_multi_contact_point_model.so)

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
