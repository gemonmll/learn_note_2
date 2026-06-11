#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

# ====================================================================
# Function: build_project()
# Description: start build process
# Usage: build_project         \
#           $build_root_path   \
#           $build_target_path  \
#           $toolchain_path    \
#           $build_type
# ====================================================================
function build_project() {
    local readonly build_root_path="$1"
    local readonly build_target_path="$2"
    local readonly toolchain_path="$3"
    local readonly build_type="$4"
    local readonly module_name="$5"

    if [ ! -d "$build_root_path" ]; then
    # -d 是否为目录
        echo "${FUNCNAME}: cannot find project root '$build_root_path'" >&2
        return 1
    fi

    if [ ! -f "$toolchain_path" ]; then
    # -f 文件是否可读
        echo "${FUNCNAME}: toolchain '$toolchain_path' does not exist" >&2
        return 1
    fi

    echo "Preparing build environment..."
    create_directory "$build_target_path"

    echo "Configuring project with toolchain '$toolchain_path'..."
    #在Build目录下执行cmake –D ..
    echo "Building with configuration '$build_type'..."
    exec_command "$build_target_path" "$(which_cmake)                               \
                                        -DCMAKE_TOOLCHAIN_FILE="$toolchain_path"    \
                                        -DCMAKE_BUILD_TYPE="$build_type"            \
                                        -DCMAKE_MODULE_NAME="$module_name"          \
                                        -DCMAKE_VERBOSE_MAKEFILE=0                  \
                                        "$build_root_path"                          \
                                      "
    #在Build目录下执行make -jn
    exec_command "$build_target_path" "make -j$(get_core_number)"

    # echo "Installing to '$build_output_path'..."
    # if [ "$build_type" == "DEBUG" ]; then
    #     exec_command "$build_target_path" "make install -j$(get_core_number)"
    # else
    #     exec_command "$build_target_path" "make install/strip -j$(get_core_number)"
    # fi

    echo "Build finished"
}
