#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

#===============================================================
# Function: which_cmake()
# Description: Determine which cmake is installed on the system
# Usage: which_cmake
#===============================================================
function which_cmake () {
    local cmake_program=$(command -v cmake)
    local cmake3_program=$(command -v cmake3)
    if [ ! -z "$cmake3_program" ]; then
        echo "$cmake3_program"
        return 0
    fi
    if [ ! -z "$cmake_program" ]; then
        echo "$cmake_program"
        return 0
    fi
    echo "${FUNCNAME}: cannot find cmake program" >&2
    return 1
}
