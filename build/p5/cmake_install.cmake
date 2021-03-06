# Install script for directory: /afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/p5

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "0")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/../p5" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/../p5")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/../p5"
         RPATH "")
  ENDIF()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/../p5")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/.." TYPE EXECUTABLE FILES "/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/build/p5/p5")
  IF(EXISTS "$ENV{DESTDIR}/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/../p5" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/../p5")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/afs/cs.cmu.edu/academic/class/15662-s15-users/minwongj/p4-physics-late1/src/../p5")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

