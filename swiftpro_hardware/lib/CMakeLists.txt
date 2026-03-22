cmake_minimum_required (VERSION 3.22.0)
set (CMAKE_VERBOSE_MAKEFILE FALSE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project (libuarm
         VERSION 0.1
         DESCRIPTION "uArm SwiftPro project"
         HOMEPAGE_URL http://url/url.html
         LANGUAGES C CXX)

set (LIBUARM_SOURCES src/uarm.cc
	src/utils/utils.cc
	src/serial/serial.cc
	src/serial/impl/unix.cc
	src/serial/impl/list_ports/list_ports_linux.cc )

add_library(uarm SHARED ${LIBUARM_SOURCES})
set_target_properties(uarm PROPERTIES COMPILE_FLAGS "-fPIC -std=c++17")
target_include_directories(uarm PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
#target_link_options(uarm PUBLIC -fPIC -shared)

