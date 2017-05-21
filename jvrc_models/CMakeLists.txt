cmake_minimum_required(VERSION 2.8.3)

project(jvrc_models)

find_package(catkin REQUIRED)

catkin_package()

execute_process(
    COMMAND
    ${CMAKE_COMMAND} -E chdir ${PROJECT_SOURCE_DIR}
    make -f ${PROJECT_SOURCE_DIR}/Makefile.jvrcmodel checkout.model
    RESULT_VARIABLE
    _make_failed
)
if (_make_failed)
  message(FATAL_ERROR "Failed to checkout jvrc model: ${_make_failed}")
endif(_make_failed)

add_custom_target(
  urdf_models ALL
  SOURCES ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/JVRC-1/jvrc-1.urdf
)

add_custom_command(
  OUTPUT ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/JVRC-1/jvrc-1.urdf
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/JVRC-1
  COMMAND simtrans -i main.wrl --prefix file://${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/JVRC-1/ -o ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/JVRC-1/jvrc-1.urdf
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/model/JVRC-1
  VERBATIM
)

install(
  DIRECTORY model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".git*" EXCLUDE
)

install(
  DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
