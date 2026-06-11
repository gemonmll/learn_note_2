#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

#===============================================================
# Function: createdirectory()
# Description: Create a directory
# Usage: create_directory $directory_path
#===============================================================
function create_directory() {
    local readonly directory_path="$1"
    mkdir -p "$directory_path"
}
