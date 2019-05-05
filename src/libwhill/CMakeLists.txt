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
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})

set(${PROJECT_NAME}_DESCRIPTION "A cross-platform library for WHILL Model CR")
set(${PROJECT_NAME}_URL "https://github.com/WHILL/ros_whill")

# Try to find serial communication library https://github.com/wjwwood/serial
find_package(serial REQUIRED)

set(SOURCES 
    src/Packet.cpp 
    src/PacketParser.cpp 
    src/PacketReceiver.cpp
    src/whill_commands.cpp
    src/WHILL.cpp
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

message(STATUS "CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(BUILD_WHILL_SHARED)    
    message(STATUS "Shared library will be built")
    add_library(whill SHARED ${SOURCES})
    set_property(TARGET whill PROPERTY VERSION ${${PROJECT_NAME}_VERSON})
    set_property(TARGET whill PROPERTY SOVERSION ${${PROJECT_NAME}_VERSON_MAJOR})
    list(APPEND WHILL_TARGETS whill)
endif()
    
if(BUILD_WHILL_STATIC)    
    message(STATUS "Static library will be built")
    add_library(whill_static STATIC ${SOURCES})
    set_target_properties(whill_static PROPERTIES OUTPUT_NAME whill)
    list(APPEND WHILL_TARGETS whill_static)
endif()

foreach (target_name ${WHILL_TARGETS})
    set_target_properties(${target_name} PROPERTIES
        PUBLIC_HEADER "include/WHILL.h"
    )
endforeach()

if(BUILD_EXAMPLE)
#[WIP] build example applications here.
endif()

if(BUILD_TEST)
#[WIP] build test here.
endif()

set(CONF_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
set(CONF_LIBRARY_DIR "${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
set(CONF_LIBRARIES   "${CONF_LIBRARY_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}whill${CMAKE_SHARED_LIBRARY_SUFFIX}")
configure_file(${PROJECT_NAME}.pc.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)

message(STATUS "Targets will be installed...")
message(STATUS "CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_LIBRARY_ARCHITECTURE=${CMAKE_LIBRARY_ARCHITECTURE}")

install(TARGETS ${WHILL_TARGETS}
        EXPORT ${PROJECT_NAME}-export
        LIBRARY DESTINATION "lib/${CMAKE_LIBRARY_ARCHITECTURE}"
        ARCHIVE DESTINATION "lib/${CMAKE_LIBRARY_ARCHITECTURE}"
        PUBLIC_HEADER DESTINATION "include/${PROJECT_NAME}"
)

install(EXPORT ${PROJECT_NAME}-export
    FILE "${PROJECT_NAME}Config.cmake"
    DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(EXPORT ${PROJECT_NAME}-export
    FILE "${PROJECT_NAME}Config-version.cmake"
    DESTINATION "lib/cmake/${PROJECT_NAME}"
)

install(FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION "lib/pkgconfig"
)





