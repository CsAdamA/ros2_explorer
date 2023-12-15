MYDIR="$(basename $PWD)"

sudo rm /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
sudo cp /workspaces/$MYDIR/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params