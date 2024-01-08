sudo cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml.backup
sudo rm /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
sudo cp $PWD/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params
