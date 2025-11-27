#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

#===============================================================
# Function: get_core_number()
# Description: Show cpu core number
# Usage: get_core_number
#===============================================================
function get_core_number() {
    cat /proc/cpuinfo | grep processor | wc -l
}
