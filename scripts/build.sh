#!/bin/bash
#set -e

#SCRIPT=$(realpath "$0")
#SCRIPTS_PATH=$(dirname "$SCRIPT")
SCRIPTS_PATH=$(pwd)
source $SCRIPTS_PATH/init_workspace.sh


cd $NAVIGATION_WS_ROOT


# Set the default build type
#BUILD_TYPE=RelWithDebInfo
#BUILD_TYPE=Debug
BUILD_TYPE=Release

colcon build \
    --packages-select pangolin \
    --merge-install \
    --symlink-install \
    --cmake-args \
        "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
        "-DCMAKE_DEBUG_POSTFIX=_d" \
        "-DCMAKE_RELWITHDEBINFO_POSTFIX=_rd" \
    -Wall -Wextra -Wpedantic


# Note: compiling ORB_SLAM3 with many cores in parallel may crash sometimes. If that happens, try compiling on a single core:
#export MAKEFLAGS="-j 1"
colcon build --packages-select ORB_SLAM3 \
    --merge-install \
    --symlink-install \
    --cmake-args \
        "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
        "-DORB_SLAM3_USE_OWN_DBoW2=ON" \
        "-DORB_SLAM3_USE_OWN_g2o=ON" \
        "-DORB_SLAM3_USE_OWN_Sophus=ON" \
        "-DORB_SLAM3_BUILD_EXAMPLES=OFF" \
        "-DORB_SLAM3_BUILD_EXAMPLES_OLD=OFF" \
    -Wall -Wextra -Wpedantic

colcon build \
    --merge-install \
    --symlink-install \
    --cmake-args \
        "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
    -Wall -Wextra -Wpedantic
