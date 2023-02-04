# hector_slam

See the ROS Wiki for documentation: http://wiki.ros.org/hector_slam

## for qrcode: 
```
sudo apt install libzbar-dev
```

# Nodes and launch files
```
ros2 launch world_info tag_detectors_launch.py
```
```
ros2 run hector_trajectory_server hector_trajectory_server
```
with hector slam
```
ros2 run hector_geotiff geotiff_node
```
one time saving with slam toolbox
```
ros2 run hector_geotiff geotiff_saver
```