cmake_minimum_required(VERSION 2.8)
project(libwhill)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

if (NOT CMAKE_BUILD_TARGET)
    message(STATUS "No target type selected, default to both shared and static libraries")
    set(CMAKE_BUILD_TARGET "Both" CACHE STRING "" FORCE)
endif()

option(BUILD_EXAMPLE "Build example program" OFF)
option(BUILD_TEST "Build test program" OFF)

set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 0)
set(${PROJECT_NAME}_VERSION_PATCH 0)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERION_PATCH})

set(${PROJECT_NAME}_DESCRIPTION "A cross-platform library for WHILL Model CR")
set(${PROJECT_NAME}_URL "https://github.com/WHILL/ros_whill")

# Try to find serial communication library https://github.com/wjwwood/serial
find_package(serial)
if (NOT serial_FOUND)
#[WIP] build serial package here
endif()

set(SOURCES 
    Packet.cpp 
    PacketParser.cpp 
    PacketReceiver.cpp
    whill_commands.cpp
    WHILL.cpp
)

if(${CMAKE_BUILD_TARGET} MATCHES "Shared")
    set(BUILD_WHILL_SHARED TRUE)
elseif(${CMAKE_BUILD_TARGET} MATCHES "Static")
    set(BUILD_WHILL_STATIC TRUE)
elseif(${CMAKE_BUILD_TARGET} MATCHES "Both")
    set(BUILD_WHILL_SHARED TRUE)
    set(BUILD_WHILL_STATIC TRUE)
else()
    message(FATAL_ERROR "Invalid build type ${CMAKE_BUILD_TYPE}")
endif()

if(BULD_WHILL_SHARED)    
    add_library(whill SHARED ${SOURCES})
    set_property(TARGET whill PROPERTY VERSION ${${PROJECT_NAME}_VERSON})
    set_property(TARGET whill PROPERTY SOVERSION ${${PROJECT_NAME}_VERSON_MAJOR})
    list(APPEND WHILL_TARGETS whill)
endif()
    
if(BULD_WHILL_STATIC)    
    add_library(whill_static STATIC ${SOURCES})
    set_target_properties(whill_static PROPERTIES OUTPUT_NAME whill)
    list(APPEND WHILL_TARGETS whill_static)
endif()





