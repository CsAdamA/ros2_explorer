#!/bin/bash
#set -e

#SCRIPT=$(realpath "$0")
#SCRIPTS_PATH=$(dirname "$SCRIPT")
#SCRIPTS_PATH=$(pwd)
#source $SCRIPTS_PATH/init_workspace.sh


#cd $NAVIGATION_WS_ROOT


# Set the default build type
#BUILD_TYPE=RelWithDebInfo
BUILD_TYPE=Release

colcon build \
    --packages-select pangolin \
    --merge-install \
    --symlink-install \
    --cmake-args \
        "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
    -Wall -Wextra -Wpedantic

colcon build --packages-select ORB_SLAM3 \
    --merge-install \
    --symlink-install \
    --cmake-args \
        "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
        "-DORB_SLAM3_USE_OWN_DBoW2=ON" \
        "-DORB_SLAM3_USE_OWN_g2o=ON" \
        "-DORB_SLAM3_USE_OWN_Sophus=ON" \
    -Wall -Wextra -Wpedantic

colcon build --packages-select turtlebot3 turtlebot3_simulations \
    --merge-install \
    --symlink-install \

colcon build \
    --merge-install \
    --symlink-install \
    --cmake-args \
        "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
    -Wall -Wextra -Wpedantic
