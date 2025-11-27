# ===============================================================================
# DESCRIPTION: Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# ===============================================================================

# Include guard
include_guard(GLOBAL)

# -----------------------------
# Macro print_build_info
# -----------------------------
macro(print_build_info)
    message(STATUS "******************************************")
    message(STATUS "***** Build configuration *****")
    message(STATUS "******************************************")
    message(STATUS "TARGET_SYSTEM       = ${TARGET_SYSTEM_NAME}")
    message(STATUS "TARGET_ARCH         = ${TARGET_ARCH}")
    message(STATUS "BUILD_TYPE          = ${CMAKE_BUILD_TYPE}")
    message(STATUS "COMPILER_C          = ${CMAKE_C_COMPILER}")
    message(STATUS "COMPILER_CXX        = ${CMAKE_CXX_COMPILER}")
    message(STATUS "COMPILE_FLAGS_C     = ${CMAKE_C_FLAGS}")
    message(STATUS "COMPILE_FLAGS_CXX   = ${CMAKE_CXX_FLAGS}")
    message(STATUS "LINK_FLAGS_C        = ${CMAKE_C_LINK_FLAGS}")
    message(STATUS "LINK_FLAGS_CXX      = ${CMAKE_CXX_LINK_FLAGS}")
    message(STATUS "INSTALL_PREFIX      = ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "******************************************")
endmacro()
