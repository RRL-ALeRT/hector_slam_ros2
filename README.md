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

# Yolov5 object detection with openvino with GPU
why? https://learnopencv.com/running-openvino-models-on-intel-integrated-gpu
## Instructions
### To get intel integrated GPU to work
Follow https://dgpu-docs.intel.com/installation-guides/ubuntu/ubuntu-jammy-arc.html
### Install openvino 
```
cd
wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2022.3/linux/l_openvino_toolkit_ubuntu20_2022.3.0.9052.9752fafe8eb_x86_64.tgz
tar -xvzf l_openvino_toolkit_ubuntu20_2022.3.0.9052.9752fafe8eb_x86_64.tgz
rm l_openvino_toolkit_ubuntu20_2022.3.0.9052.9752fafe8eb_x86_64.tgz
mv l_openvino_toolkit_ubuntu20_2022.3.0.9052.9752fafe8eb_x86_64 openvino2022.3
. ~/openvino2022.3/setupvars.sh
echo '
#OpenVINO
. ~/openvino2022.3/setupvars.sh > /dev/null' >> ~/.bashrc
```
