#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

#===============================================================
# Function: update_timestamp()
# Description: Update all files' timestamp in a directory
# Usage: update_timestamp $directory
#===============================================================
function update_timestamp() {
    local readonly directory="$1"
    if [ ! -d "$directory" ]; then
        echo "${FUNCNAME}: cannot find directory '$directory'" >&2
        return 1
    fi
    touch $(find "$directory")
#将输入目录的时间戳修改为当前系统的时间戳
}
