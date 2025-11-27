#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.0
# ===============================================================================

# script options
set -e -o nounset

# 当前脚本所在绝对路径
readonly BUILD_PATH=$(dirname $(readlink -f $0))

# 项目跟目录
readonly PROJECT_ROOT=$(readlink -f $BUILD_PATH/..)

readonly SCRIPT_FOLDER=$PROJECT_ROOT/build/scripts
readonly CMAKE_FOLDER=$PROJECT_ROOT/build/cmake
readonly SOURCE_FOLDER=$PROJECT_ROOT
readonly TARGET_FOLDER=$PROJECT_ROOT/target     # 临时编译文件所在路径
readonly RUNNING_FOLDER=$PROJECT_ROOT/running

# default settings
readonly DEFAULT_TOOLCHAIN_NAME=linux

# ===============================================================
# Function: main()
# Description: script main entery
# Usage: build.sh $toolchain_name $build_type(debug/release)
# ===============================================================
function main() {
    local readonly script_name=$(basename $0)
    local toolchain_path="";
    local build_type="debug";
    local module_name="all";

    if [ $# -eq 1 ] && [ "$1" = "clean" ]; then
        echo "Clean the target directory..."
        clean
        return 0
    fi

    if [ $# -gt 3 ]; then
        echo "Usage: $script_name [toolchain_name] [debug/release] [module_name]" >&2
        return 1
    fi

    echo "#==============================================================================="
    echo "# Unified build framework"
    echo "# Copyright (c) 2019 Faw Technologies Co., Ltd. All rights reserved."
    echo "#==============================================================================="

    if [ $# -eq 1 ]; then
        toolchain_path="$CMAKE_FOLDER/$1.cmake"
    elif [ $# -eq 2 ]; then
        toolchain_path="$CMAKE_FOLDER/$1.cmake"
        build_type="$2"
    elif [ $# -eq 3 ]; then
        toolchain_path="$CMAKE_FOLDER/$1.cmake"
        build_type="$2"
        module_name="$3"
    fi

    if [ -z "$toolchain_path" ]; then
        echo "Using default toolchain '$DEFAULT_TOOLCHAIN_NAME'"
        toolchain_path="$CMAKE_FOLDER/$DEFAULT_TOOLCHAIN_NAME.cmake"
    fi

    update_timestamp "$SOURCE_FOLDER"
    build_project    "$PROJECT_ROOT"                \
                     "$TARGET_FOLDER"               \
                     "$toolchain_path"              \
                     $(to_upper_case "$build_type") \
                     "$module_name"
}

source "$SCRIPT_FOLDER/bootstrap.sh"
bootstrap  "$SCRIPT_FOLDER"
#执行函数bootstrap
main $@

