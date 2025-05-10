
# Bluerobotics Ping360 Scanning Imaging Sonar ROS2 Driver

### Installzation
```sh
# download
git clone https://github.com/GSO-soslab/bluerobotics_ping360 --recursive
# switch to jazzy-devel branch
git checkout jazzy-devel
# build
cd ~/your_ros_ws
colcon build --packages-select ping360_sonar ping360_msgs
# launch
ros2 launch ping360_sonar ping360_sonar.launch.py
```

### Acknowledgement
This ROS2 package is derived from [ping360 sonar ROS2 driver](https://github.com/CentraleNantesRobotics/ping360_sonar). thanks for their awesome work !!