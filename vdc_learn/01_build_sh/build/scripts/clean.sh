#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

#===============================================================
# Function: clean()
# Description: clean the content in output folder and logs
# Usage: remove $path
#===============================================================
function clean() {
    remove "$TARGET_FOLDER/"
    remove "$RUNNING_FOLDER/"
    rm running.tar.gz
    return 0
}
