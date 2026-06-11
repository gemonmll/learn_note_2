#!/bin/bash
# ===============================================================================
# Unified build framework
# Copyright (c) 2019 Huawei Technologies Co., Ltd. All rights reserved.
# VERSION: 1.0
# ===============================================================================

# script options
set -e -o nounset

# current script information
readonly SCRIPT_PATH=$(dirname $(readlink -f $0))
readonly ROOT_PATH=${SCRIPT_PATH}/..
readonly RUN_DIR="running"
readonly RUN_ROOT_PATH=${ROOT_PATH}/${RUN_DIR}
# readonly app_list=("SystemMgr")

# ===============================================================
# Function: main()
# Description: script main entery
# Usage: build.sh $toolchain_name $build_type(debug/release)
# ===============================================================
function main() {
    echo ${ROOT_PATH}
    cd ${ROOT_PATH}
    if [ ! -d "${RUN_DIR}" ]; then
        mkdir ${RUN_DIR}
    fi

    cd ${RUN_DIR}
     if [ -d "etc" ]; then
         rm -r ${RUN_ROOT_PATH}/etc
     fi
     if [ -d "lib" ]; then
         rm -r ${RUN_ROOT_PATH}/lib
     fi
     if [ -d "include" ]; then
         rm -r ${RUN_ROOT_PATH}/include
     fi
     if [ -d "doc" ]; then
         rm -r ${RUN_ROOT_PATH}/doc
     fi
     if [ -d "script" ]; then
         rm -r ${RUN_ROOT_PATH}/script
     fi
     if [ -d "version" ]; then
         rm -r ${RUN_ROOT_PATH}/version
     fi
     if [ -d "man" ]; then
         rm -r ${RUN_ROOT_PATH}/man
     fi
     if [ -d "boot" ]; then
         rm -r ${RUN_ROOT_PATH}/boot
     fi
     if [ -d "signature" ]; then
         rm -r ${RUN_ROOT_PATH}/signature
     fi
    
    mkdir AppM
    mkdir AppM/Sf_VCt_L2_ParkingAppM AppM/Sf_VCt_L2_ParkingAppM/bin AppM/Sf_VCt_L2_ParkingAppM/etc
  #  mkdir AppM/Sf_VCo_L2_ParkingFus AppM/Sf_VCo_L2_ParkingFus/bin AppM/Sf_VCo_L2_ParkingFus/etc
  #  mkdir AppM/Sf_VM_L2_UIUE AppM/Sf_VM_L2_UIUE/bin AppM/Sf_VM_L2_UIUE/etc
   # mkdir AppM/Sf_VCt_L2_USSPerc AppM/Sf_VCt_L2_USSPerc/bin AppM/Sf_VCt_L2_USSPerc/etc
  #  mkdir AppM/Sf_VM_L2_ParkingPlan AppM/Sf_VM_L2_ParkingPlan/bin AppM/Sf_VM_L2_ParkingPlan/etc
    mkdir  lib 

#系统管理
    if [ -f ${ROOT_PATH}/target/AppM/Sf_VCt_L2_ParkingAppM/src/libL2_ParkingAppM.so ];then
        cp -r ${ROOT_PATH}/target/AppM/Sf_VCt_L2_ParkingAppM/src/libL2_ParkingAppM.so      ${RUN_ROOT_PATH}/lib
    fi
    if [ -f ${ROOT_PATH}/target/AppM/Sf_VCt_L2_ParkingAppM/Sf_VCt_L2_ParkingAppM ];then
        cp -r ${ROOT_PATH}/target/AppM/Sf_VCt_L2_ParkingAppM/Sf_VCt_L2_ParkingAppM      ${RUN_ROOT_PATH}/AppM/Sf_VCt_L2_ParkingAppM/bin/
    fi

#标定动态库    
    cp -r ${ROOT_PATH}/third_part/calib_api/libParsing_Parking_CalibParams.so      ${RUN_ROOT_PATH}/lib
    cp -r ${ROOT_PATH}/AppM/Sf_VCt_L2_ParkingAppM/lib/libfaw_calibdata.so      ${RUN_ROOT_PATH}/lib

#配置文件
  #  cp -r ${ROOT_PATH}/AppM/Sf_VCo_L2_ParkingFus/etc/*       ${RUN_ROOT_PATH}/AppM/Sf_VCo_L2_ParkingFus/etc/
  #  cp -r ${ROOT_PATH}/AppM/Sf_VCt_L2_USSPerc/etc/*          ${RUN_ROOT_PATH}/AppM/Sf_VCt_L2_USSPerc/etc/
  #  cp -r ${ROOT_PATH}/AppM/Sf_VM_L2_UIUE/etc/*              ${RUN_ROOT_PATH}/AppM/Sf_VM_L2_UIUE/etc/
  #  cp -r ${ROOT_PATH}/AppM/Sf_VM_L2_ParkingPlan/etc/*       ${RUN_ROOT_PATH}/AppM/Sf_VM_L2_ParkingPlan/etc/
    cp -r ${ROOT_PATH}/AppM/Sf_VCt_L2_ParkingAppM/etc/*      ${RUN_ROOT_PATH}/AppM/Sf_VCt_L2_ParkingAppM/etc/
   
#中间件框架动态库
    if [ $1 == "linux" ]; then
        cp ${ROOT_PATH}/middleware/lib_linux/*   ${RUN_ROOT_PATH}/lib
    else
        cp ${ROOT_PATH}/middleware/lib_aos/*     ${RUN_ROOT_PATH}/lib
    fi
    
    # cp -r ${ROOT_PATH}/build/running_env.sh      ${RUN_ROOT_PATH}/script
    # cp -r ${ROOT_PATH}/build/running_start.sh      ${RUN_ROOT_PATH}/script

    tar -zcvf ${ROOT_PATH}/running.tar.gz ${ROOT_PATH}/running
}

main $@
