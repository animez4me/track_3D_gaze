cmake_minimum_required(VERSION 2.8.11)
project(track_3D_gaze)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
#  INCLUDE_DIRS include
#  LIBRARIES track_3D_gaze
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/track_3D_gaze.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(3d_gaze_node
#    src/track_gaze.cpp
#    src/track.cpp
#    src/RGB.cpp
#    src/RGBD.cpp
#    src/matcher.cpp
#    src/EyeTracking_2D3D.cpp
#    src/Utils.cpp
#    src/Mesh.cpp
#    )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(3d_gaze_node
#   ${catkin_LIBRARIES}
# )


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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_track_3D_gaze.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


include(CheckIncludeFiles)

#project(brisk_pnp CXX)



set(GCC_COVERAGE_COMPILE_FLAGS "-Wno-reorder -Wno-sign-compare -Wno-unused-value -Wno-unused-variable -Wno-unused-but-set-variable -Wall -Wno-comment")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE  Release)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(INCLUDE           "${CMAKE_SOURCE_DIR}/include")

# The version number
set(BRISK_MAJOR_VERSION 2)
set(BRISK_MINOR_VERSION 0)
set(BRISK_PATCH_VERSION 2)
set(BRISK_VERSION
        ${BRISK_MAJOR_VERSION}.${BRISK_MINOR_VERSION}.${BRISK_PATCH_VERSION})

set(FREENECT2_INCLUDE_DIRS "~/freenect2/include")
set(FREENECT2_LIBRARY "~/freenect2/lib/libfreenect2.so")

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/brisk)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# make sure we use Release and warn otherwise
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(WARNING "CMAKE_BUILD_TYPE not set to 'Release'. Performance may be terrible.")
else()
  message(STATUS "Building with build type '${CMAKE_BUILD_TYPE}'")
endif()

# require OpenCV
find_package( OpenCV REQUIRED )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})
find_package(freenect2 REQUIRED)
include_directories(${FREENECT2_INCLUDE_DIRS})
include_directories(${INCLUDE})
#find_package( OpenGV REQUIRED)

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm*")
  message(STATUS "ARM processor detected, will attempt to use NEON.")
  add_definitions(-mfpu=neon -DHAVE_OPENCV -Wall -pedantic -std=c++0x -fPIC)
else()
  message(STATUS "Assuming SSE instructions available.")
  add_definitions(-mssse3 -DHAVE_OPENCV -Wall -pedantic -std=c++0x -fPIC)
endif()

# set up the output tree
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# check include file safely
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  if(EXISTS "${dir}/opencv2/nonfree/nonfree.hpp")
    set(HAVE_OPENCV_NONFREE_H TRUE)
    #message(STATUS "Found ${dir}/opencv2/nonfree/nonfree.hpp, setting HAVE_OPENCV_NONFREE_H=TRUE")
  endif()
endforeach()
if(${OpenCV_VERSION_MAJOR} STREQUAL 3)
  message(WARNING "Nonfree module disabled, demo application with limited functionality")
endif()

if(NOT APPLE)
  # The clang compiler (on osx) is somehow much more strict
  # than the compilers on ubuntu and so this does not seem
  # possible on OSX just yet.
  add_definitions( -Werror )
endif()

option(WITH_PCL "adds pcl cloud support" ON)
if(${WITH_PCL})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_PCL")
    find_package(PCL REQUIRED)
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    include_directories(${PCL_INCLUDE_DIRS})
endif()

# the absolute path to the demo images
set(BRISK_IMAGE_PATH \"${PROJECT_SOURCE_DIR}/images\")
add_definitions(-DBRISK_IMAGE_PATH=${BRISK_IMAGE_PATH})

# build agast
include_directories(agast/include)
include_directories(include)
add_subdirectory(agast)

# build brisk
add_library(${PROJECT_NAME} STATIC
                            src/brisk/brisk-descriptor-extractor.cc
                            src/brisk/brisk-feature-detector.cc
                            src/brisk/brisk-layer.cc
                            src/brisk/brisk-scale-space.cc
                            src/brisk/brute-force-matcher.cc
                            src/brisk/harris-feature-detector.cc
                            src/brisk/harris-score-calculator.cc
                            src/brisk/harris-score-calculator-float.cc
                            src/brisk/harris-scores.cc
                            src/brisk/image-down-sampling.cc
                            src/brisk/pattern-provider.cc
                            src/brisk/vectorized-filters.cc)

# and link it

target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} agast)
#add_executable(track_app src/track.cpp src/RGB.cpp src/RGBD.cpp src/matcher.cpp src/EyeTracking_2D3D.cpp src/Utils.cpp src/Mesh.cpp)
include_directories(include)
add_executable(track_3d_gaze_node
    #src/track_gaze.cpp
    src/track.cpp
    src/RGB.cpp
    src/RGBD.cpp
    src/matcher.cpp
    src/EyeTracking_2D3D.cpp
    src/Utils.cpp
    src/Mesh.cpp
    #include/track.h
    include/RGB.h
    include/RGBD.h
    include/matcher.h
    include/EyeTracking_2D3D.h
    include/Utils.h
    include/Mesh.h
    )

target_link_libraries(track_3d_gaze_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${FREENECT2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})
#target_link_libraries(3d_gaze_node  ${FREENECT2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS})

## installation if required
#install(TARGETS ${PROJECT_NAME} EXPORT briskTargets ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
##install(TARGETS track_app EXPORT briskTargets DESTINATION ${INSTALL_BIN_DIR})
#install(TARGETS 3d_gaze_node EXPORT briskTargets DESTINATION ${INSTALL_BIN_DIR})
#install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.h")

## installation is invoked in the individual modules...
#export (TARGETS
#    brisk_pnp
#    agast
#    #track_app
#    3d_gaze_node
#    FILE "${PROJECT_BINARY_DIR}/briskTargets.cmake")
#export ( PACKAGE brisk )

## Create the briskConfig.cmake and briskConfigVersion files
#file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
#   "${INSTALL_INCLUDE_DIR}")
## ... for the build tree
#set(CONF_INCLUDE_DIRS
#  "${PROJECT_SOURCE_DIR}/include"
#  "${PROJECT_BINARY_DIR}"
#)
#configure_file(cmake/briskConfig.cmake.in
#  "${PROJECT_BINARY_DIR}/briskConfig.cmake" @ONLY)
## ... for the install tree
#set(CONF_INCLUDE_DIRS "\${BRISK_CMAKE_DIR}/${REL_INCLUDE_DIR}")
#configure_file(cmake/briskConfig.cmake.in
#  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/briskConfig.cmake" @ONLY)
## ... for both
#configure_file(cmake/briskConfigVersion.cmake.in
#  "${PROJECT_BINARY_DIR}/briskConfigVersion.cmake" @ONLY)

## Install the briskConfig.cmake and briskConfigVersion.cmake
#install(FILES
#  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/briskConfig.cmake"
#  "${PROJECT_BINARY_DIR}/briskConfigVersion.cmake"
#  DESTINATION "${INSTALL_CMAKE_DIR}")

## Install the export set for use with the install-tree
#install(EXPORT briskTargets DESTINATION
#  "${INSTALL_CMAKE_DIR}")

