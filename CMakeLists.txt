# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(max30102_demo)

nanopb_generate_cpp(proto_sources proto_headers RELPATH .
    src/simple.proto
)
# we need to be able to include generated header files
zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})

target_sources(app PRIVATE src/main.c)

