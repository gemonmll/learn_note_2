#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.4
# ===============================================================================
set -e

#===============================================================
# Function: to_upper_case()
# Description: Show upper case of inputed string
# Usage: get_core_number $input
#===============================================================
function to_upper_case() {
    echo "$1" | tr '[a-z]' '[A-Z]'
}
