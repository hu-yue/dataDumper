# Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Marco Randazzo - marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.0)
SET(PROJECTNAME DataDumper)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

set(HEADERS_DIR include/dataDumper)

FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# import math symbols from standard cmath
#add_definitions(-D_USE_MATH_DEFINES)

INCLUDE_DIRECTORIES(${HEADERS_DIR} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} iKin iDyn)
