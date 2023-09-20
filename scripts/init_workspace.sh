#!/bin/bash

#SCRIPT=$(realpath "$0")
#SCRIPTS_PATH=$(dirname "$SCRIPT")
SCRIPTS_PATH=$(pwd)
export NAVIGATION_WS_ROOT=$SCRIPTS_PATH/..

source /opt/ros/humble/setup.bash
if [ -f ${NAVIGATION_WS_ROOT}/install/setup.bash ]; then
    source ${NAVIGATION_WS_ROOT}/install/setup.bash;
fi
