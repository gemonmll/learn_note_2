#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

#===============================================================
# Function: remove()
# Description: remove a file or directory
# Usage: remove $path
#===============================================================
function remove() {
    local readonly path="$1"
    if [ "$path" == "/" ]; then
        echo "${FUNCNAME}: cannot remove root directory '/'" >&2
        return 1
    fi
    
    [ -n "$path" ] && rm -rf "$path"
}
