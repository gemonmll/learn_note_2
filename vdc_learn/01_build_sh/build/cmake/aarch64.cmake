# ===============================================================================
# DESCRIPTION: Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# ===============================================================================

# Include guard
include_guard(GLOBAL)

# Include macros
include(${CMAKE_CURRENT_LIST_DIR}/macros/print_project_logo.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/macros/print_build_info.cmake)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/middleware/lib_aos)
# Show logo
print_project_logo()

#set(AOS_SDK_PATH $ENV{AOS_SDK_PATH})
set(AOS_SDK_PATH /home/faw/tda4_sdk/SDK)
if (NOT DEFINED AOS_SDK_PATH)
    message(WARNING "Can't find 'AOS_SDK_PATH', please check")
else()
    message(STATUS "AOS_SDK_PATH is set, PATH is $ENV{AOS_SDK_PATH}")
endif()

set(CMAKE_CROSSCOMPILING TRUE)
set(TOOLCHAIN_PATH ${AOS_SDK_PATH})

# 设置交叉编译sysroot，CMAKE_SYSROOT只有在3.0以上的版本才有效
set(CMAKE_SYSROOT "${TOOLCHAIN_PATH}/aos/arm64/common/sysroot")
set(BUILD_SYSROOT "${TOOLCHAIN_PATH}/aos/arm64/common//sysroot")

# Toolchain settings
set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/aos/arm64/common/bin/clang)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/aos/arm64/common/bin/clang++)
set(CMAKE_C_LINK_EXECUTABLE   "<CMAKE_C_COMPILER>   <FLAGS> <CMAKE_C_LINK_FLAGS>   <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")

set(CMAKE_FIND_ROOT_PATH "${TOOLCHAIN_PATH}")

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
set(TARGET_SYSTEM_NAME EulerOS)
set(TARGET_ARCH        aarch64)

# Print build info
print_build_info()
