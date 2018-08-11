cmake_minimum_required(VERSION 2.8)
project(libccd C)

set(CCD_VERSION "2.0")
set(CCD_SOVERSION "2")
set(PKG_DESC "Library for collision detection between convex shapes")

# Include GNUInstallDirs to get canonical paths
include(GNUInstallDirs)

include_directories("src/")

# make sure only DOUBLE or SINGLE is set; default to SINGLE
if (${CCD_SINGLE})
  set(CCD_DOUBLE false)
else()
  set(CCD_SINGLE true)
endif()

if (${CCD_DOUBLE})
  set(CCD_SINGLE false)
endif()

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/ccd/config.h.cmake.in" "${CMAKE_CURRENT_SOURCE_DIR}/src/ccd/config.h")

if(NOT WIN32)
    find_library(MATH m)
else()
    set(MATH "")
endif()

set(SOURCE_FILES
  src/ccd.c
  src/mpr.c
  src/polytope.c
  src/support.c
  src/vec3.c)

add_library(ccd SHARED ${SOURCE_FILES})
set_target_properties(ccd PROPERTIES VERSION ${CCD_VERSION} SOVERSION ${CCD_SOVERSION})

target_link_libraries(ccd ${MATH})

add_library(ccd_static STATIC ${SOURCE_FILES})
set_target_properties(ccd_static PROPERTIES OUTPUT_NAME ccd)
target_link_libraries(ccd_static ${MATH})

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/ccd.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

install(TARGETS ccd ccd_static ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY src/ccd DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES "${pkg_conf_file}" DESTINATION
  ${CMAKE_INSTALL_LIBDIR}/pkgconfig COMPONENT pkgconfig)
