#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

#===============================================================
# Function: exec_command()
# Description: Execute a command in a directory
# Usage: exec_command $working_directory $command
#===============================================================
function exec_command() {
    local readonly working_directory="$1"
    local readonly command="$2"
    if [ ! -d "$working_directory" ]; then
        echo "${FUNCNAME}: cannot find working directory '$working_directory'" >&2
        return 1
    fi
    pushd "$working_directory" > /dev/null
    $command
    popd > /dev/null
}
