# Frontier explorer

Open docker container, then replace nav2_params.yaml:
   ```
   source param_replace.sh
   ```
Then to launch the explorer:
1. terminal:
   ```
   source config.sh
   ros2 launch turtlebot4_navigation slam.launch.py
   ```
2. terminal:
   ```
   source config.sh
   ros2 launch turtlebot4_navigation nav2.launch.py
   ```
3. terminal:
   ```
   source config.sh
   ros2 launch turtlebot4_viz view_robot.launch.py
   ```
Only launch explorer in 4. terminal when nav2 is active and slam map is visible in rviz.

4. terminal:
   ```
   source config.sh
   /bin/python3 /workspaces/nav_ws_humble3/install/nav2_wfd/lib/nav2_wfd/explore
   ```
