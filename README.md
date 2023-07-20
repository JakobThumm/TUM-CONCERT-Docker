# TUM-CONCERT-Docker
Docker files for CONCERT Integration of TUM

## BUILD AND RUN THE DOCKER CONTAINER
```
./docker/build-docker.bash --no-cache 
./docker/run-docker.bash
```
Builds the image `jakobthumm/tum-concert:latest.`

## Inside Docker

### Set xbot2 config file
Right now, we have to do this manually: Go to
```
cd /home/user/tum_integration_ws/src/concert_description/concert_gazebo/launch
```
and edit `modular.launch`
Replace 
```
$(find concert_xbot2)/modular.yaml
```
with 
```
/home/user/tum_integration_ws/src/sara-shield/config/xbot2_config.yaml
```

### Start roscore
```
source /home/user/tum_integration_ws/setup.bash
roscore
```

### Start gazebo simulation
```
source /home/user/tum_integration_ws/setup.bash
mon launch concert_gazebo concert.launch
```

### Open `xbot2-Gui`
**After** Gazebo runs, start Xbot2-Gui with
```
xbot2-gui
```
This should open a large window with status "Running" in the top left corner. If it opens a small window with the status "Inactive" instead, close and rerun the command above.

### Open `rviz`
**Open Rviz** for visualization 
```
source /home/user/tum_integration_ws/setup.bash
rviz
```
and **start the testnode plugin** (will be renamed to safety-shield) in the xbot2 gui. 

TODO: upload rviz config

The visualization topics of sara-shield are named ```/human_joint_marker_array``` and ```/robot_joint_marker_array``` and can be visualized by adding them in rviz.

## Send goal position:
```
rostopic pub /goal_joint_pos std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [-2.5, 1.4, 0.0, 0.0, 0.0, 0.0]"
```
and then
```
rostopic pub /goal_joint_pos std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [2.5, 1.4, 0.0, 0.0, 0.0, 0.0]"
```
## Send safety override
To set the safety shield to unsafe send
```
rostopic pub /safe_flag std_msgs/Bool "data: false"
```
send true to activate normal safety shield behavior again.

## Send dummy human measurements
Send dummy measurements:
```
rostopic pub /sara_shield/send_dummy_meas std_msgs/Bool "data: true"
```
Don't send dummy measurements:
```
rostopic pub /sara_shield/send_dummy_meas std_msgs/Bool "data: false"
```
