#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

# ===============================================================
# Function: bootstrap()
# Description: Load all script command from a directory
# Usage: bootstrap $script_directory
# ===============================================================
function bootstrap() {
    echo "Bootstrapping..."
    local readonly script_directory="$1"
    if [ ! -d "$script_directory" ]; then
        echo "${FUNCNAME}: cannot find script directory '$script_directory'" >&2
        return 1
    fi

    local readonly script_list=$(find "$script_directory" -name "*.sh")
    if [ -z "$script_list" ]; then
        echo "${FUNCNAME}: cannot find any script under directory '$script_directory'" >&2
        return 1
    fi

    for script in $script_list; do
        # we don't source a script file w/ execution permission
        # cuz it is an external command
        # Besides, we don't source this file as well
        if [ -x "$script" ] && [ "$script" == "${BASH_SOURCE[0]}" ]; then
            continue;
        fi
        source "$script"
    done
}
