# ===============================================================================
# DESCRIPTION: Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# ===============================================================================

# Include guard
include_guard(GLOBAL)

# Include macros
include(${CMAKE_CURRENT_LIST_DIR}/macros/print_project_logo.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/macros/print_build_info.cmake)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/middleware/lib_linux)
# Show logo
print_project_logo()

set(BUILD_SYSROOT "")
# Toolchain settings
set(CMAKE_SYSTEM_NAME         Linux)
set(CMAKE_C_COMPILER          $ENV{CC})
set(CMAKE_CXX_COMPILER        $ENV{CXX})
set(CMAKE_C_LINK_EXECUTABLE   "<CMAKE_C_COMPILER>   <FLAGS> <CMAKE_C_LINK_FLAGS>   <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")

# Build flags
set(CMAKE_C_FLAGS
    ${CMAKE_C_FLAGS}
    ${TOOLCHAIN_DEFAULT_C_FLAGS}
)

set(CMAKE_CXX_FLAGS
    ${CMAKE_CXX_FLAGS}
    ${TOOLCHAIN_DEFAULT_CXX_FLAGS}
)

# Link flags
set(CMAKE_C_LINK_FLAGS
    ${CMAKE_C_LINK_FLAGS}
    ${TOOLCHAIN_DEFAULT_C_LINK_FLAGS}
)

set(CMAKE_CXX_LINK_FLAGS
    ${CMAKE_CXX_LINK_FLAGS}
    ${TOOLCHAIN_DEFAULT_CXX_LINK_FLAGS}
)

string(REPLACE ";" " " CMAKE_C_FLAGS        "${CMAKE_C_FLAGS}")
string(REPLACE ";" " " CMAKE_CXX_FLAGS      "${CMAKE_CXX_FLAGS}")
string(REPLACE ";" " " CMAKE_C_LINK_FLAGS   "${CMAKE_C_LINK_FLAGS}")
string(REPLACE ";" " " CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS}")

# Project build settings
set(TARGET_SYSTEM_NAME Linux)
set(TARGET_ARCH        linux)

# Print build info
print_build_info()
