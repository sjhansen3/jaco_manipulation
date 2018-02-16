
# Install

# Examples

## Fake robot
`roslaunch manipulation.launch`
Which does does the following:
#TODO needs testing: does the finger action server work in sim?

1. **load the planning context** Load the URDF, SRDF and other .yaml configuration files on the param server
2. **fake joint state publisher** We do not have a robot connected, so publish fake joint states
3. **Robot state publisher** Given the published joint states, publish tf for the robot links
4. **launch rviz** With manipulation_config.rviz as the coniguration file

## load drivers and planner for real robot
1. Launch the **kinova_driver** node to control the arm.

`roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2s7s300 /joint_states:=/j2s7s300_driver/out/joint_state`

2. run **rviz** with manipulation.launch.

`rosrun rviz rviz`

3. Load extra drivers for fingers and **moveit**.

`roslaunch j2s7s300_moveit_config j2s7s300_demo.launch /joint_states:=/j2s7s300_driver/out/joint_state`
Which loads the following:
* move_group node, with controllers, defined in #TODO fill this in/config/controllers.yaml
* joint_trajectory_action_server node available in kinova_driver
* gripper_command_action_server node available in kinova_driver
* rviz

## Command line tool for saving poses

`rosrun jaco_manipulation store_poses.py /joint_states:=/j2s7s300_driver/out/joint_state`


## loading AR Trackers
`roslaunch jaco_manipulation ar_track_alavar.launch`
1. **launch AR tracker** indigo branch of ar_track_alvar https://github.com/ros-perception/ar_track_alvar/tree/indigo-devel
2. **launch kinect bridge** iai_kinect2 from https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge
3. **calibrate the kinect** using calibrate_camera.py
* this requries ar_tracker_6 be placed right below the robot

#TODO Point clouds are only published when the launch file is used. Run roslaunch kinect2_bridge kinect2_bridge.launch

for debugging kinect connection issues:
While bridge is running, run the following to view the camera
`rosrun kinect2_viewer kinect2_viewer hd image `

A ros independent way to check kinect drivers is by running
`./libfreenect2/build/bin/Protonect` from the home directory


## Run pick and place demo
`rosrun jaco_manipulation moveit_interface.py /joint_states:=/j2s7s300_driver/out/joint_state`
Requires the following to be loaded:
* AR Trackers
* MoveIt!
* Bringup

#TODO test if this works in sim

## Dependencies

#TODO need more documentation here

src containst the following:
* ar_track_alvar
* autolab_core
* iai_kinect2
* jaco_manipulation
* kinova-ros
* perception
