source /opt/ros/humble/setup.bash
source /workspaces/navigation_ws_humble1/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/workspaces/navigation_ws_humble1/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py