# Install script for directory: /mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/src/project1_turtlesim

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/build/project1_turtlesim/catkin_generated/installspace/project1_turtlesim.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/project1_turtlesim/cmake" TYPE FILE FILES
    "/mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/build/project1_turtlesim/catkin_generated/installspace/project1_turtlesimConfig.cmake"
    "/mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/build/project1_turtlesim/catkin_generated/installspace/project1_turtlesimConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/project1_turtlesim" TYPE FILE FILES "/mnt/host/Desktop/Seneca_Class_Notes/Semester 2/AIG240 - Robotics/ros_ws/catkin_ws/src/project1_turtlesim/package.xml")
endif()

