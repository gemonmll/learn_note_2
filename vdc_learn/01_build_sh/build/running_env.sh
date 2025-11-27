#!/bin/bash

CURRENT_DIR=$(cd $(dirname $0); pwd)
export AD_HOME=$CURRENT_DIR
export AD_DATA=$CURRENT_DIR
export LD_LIBRARY_PATH=$AD_HOME/lib:$LD_LIBRARY_PATH


#killall someipd
#export SOMEIP_CONFIG_FILE=$AD_HOME/etc/someipd-tda4_b.json
#/bin/someipd &

