#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

# ===============================================================
# Function: apply_patch()
# Description: Search recursively for all patch files in the DIR
# Usage: apply_patch $patch_directory
# ===============================================================
function apply_patch() {
    local readonly script_directory="$1"
    if [ ! -d "$script_directory" ]; then
        echo "$FUNCNAME: cannot find directory '$script_directory'" >&2
        return 1
    fi

    local readonly patch_list=$(find "$script_directory" -name "*.patch"|sort)
    if [ -z "$patch_list" ]; then
        echo "$FUNCNAME: cannot find any patch files under directory '$script_directory'" >&2
        return 1
    fi

    for patch in $patch_list; do
        echo "Applying path '$patch'..."
        git am "$patch" 2>/dev/null || true
    done
}
