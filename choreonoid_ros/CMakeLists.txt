# http://ros.org/doc/hydro/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
cmake_policy(VERSION ${CMAKE_MINIMUM_REQUIRED_VERSION})

if (CMAKE_VERSION VERSION_GREATER "2.9.9.9")
  cmake_policy(SET CMP0037 OLD)
endif()

project(choreonoid_ros)

find_package(catkin REQUIRED COMPONENTS mk)

catkin_package(DEPENDS eigen)

execute_process(
    COMMAND
    ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_BINARY_DIR}
    make -f ${PROJECT_SOURCE_DIR}/Makefile.choreonoid INSTALL_DIR=${CATKIN_DEVEL_PREFIX} CMD_SETTING_BUILD_ARGS=${PROJECT_SOURCE_DIR}/setting_build_args.sh installed.choreonoid
    RESULT_VARIABLE
    _make_failed
)
if (_make_failed)
  message(FATAL_ERROR "Failed to build choreonoid: ${_make_failed}")
endif(_make_failed)

add_custom_target(clean
   COMMAND
   ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_BINARY_DIR}
   make -f ${PROJECT_SOURCE_DIR}/Makefile.choreonoid INSTALL_DIR=${CATKIN_DEVEL_PREFIX} clean.choreonoid
)

install(
  DIRECTORY launch model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

#
# Following codes are to handle a build error of the Choreonoid ROS plugin
# caused by a lack of the header file which should be installed into the devel/ directory.
#  If you have a build error caused by a missing include file, uncomment the
# configure_file() and specify the missing file name.
#
execute_process(
  COMMAND find ${CATKIN_DEVEL_PREFIX}/include -name "choreonoid-*"
  OUTPUT_VARIABLE _cnoid_include_path
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
  COMMAND find ${CATKIN_DEVEL_PREFIX}/../build -name "choreonoid-master"
  OUTPUT_VARIABLE _cnoid_build_path
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if("${_cnoid_include_path}" STREQUAL "")
  message(WARNING "Chorenoid include directory not found")
elseif("${_cnoid_build_path}" STREQUAL "")
  message(WARNING "Chorenoid build directory not found")
else()
#  configure_file(
#    ${_cnoid_build_path}/src/Body/ExtraJoint.h
#    ${_cnoid_include_path}/cnoid/src/Body
#    COPYONLY
#  )
endif()
