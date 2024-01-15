**Readme for navigation of Team SWOT**

- **author:** Fabio Mast
- **contact:** fabio.mast@student.fhws.de
- **date:** 31.01.2022

**1. Start the connection with the Jetson TX2.**
  - `ssh irobot@192.168.0.2`
  - pw: myrobot 

**1.1 Load the drivers and start the bringup.**
  - `sudo ~/ws/c*/src/evo*/evo*exam*/scripts/can_init.sh`
  - `roslaunch swot_bringup bringup.launch`

**2. Start the connection with the UP Xtreme.**
  - `ssh fe-w364@192.168.0.3`
  - pw: myrobot 

**2.1 Start the Navigation** 
With the whole RoboCup structure (e.g. communication with task management etc.) use:
  - `roslaunch swot_navigation swot_robocup_navigation.launch`
When you want to use the navigation only for testing without the whole structure for RoboCup communication use:
  - `roslaunch swot_navigation navigation.launch`
Choose the right Rviz-Config: Swot_Default.rviz


**INFOS:**

Rosservice for sending the next desired target area:
  - `rosservice call /swot_next_destination "destination: ''"`
  - Possible destinations are: "WS01" - "WS06", "TT01", "SH01", "START", "END"
  - When the robot reaches the goal pose, it will return the status "FINISHED" (other status are "FAILED" and "TAPE")

How to teach new poses of the workspaces:
  - Drive to the desired goal pose in front of the Service Area 
  (Hint: Choose the pose with 20-30 cm distance to the Service Area, to be sure, that the robot can reach it!)
  - `rosrun tf tf_echo /map /base_footprint` -> get the current pose of the robot in the map
  - Insert new coordinates in WS_Positions.csv (Positon: x|y|z Orientation: x|y|z|w)


How to create a new map using slam_gmapping:
  - Connect with the UP Xtreme (look at 2.)
  - `roslaunch swot_slam slam_gmapping.launch`
  - Open Rviz to see the map you will create
  - roslaunch swot_teleop teleop.launch (manual control of the robot via keyboard)
  - When the map was created successfully: `rosrun map_server map_saver -f ~/catkin_ws/src/swot/swot_navigation/maps/"Name"`
  - in swot_navigation/navigation.launch: Choose the new map.yaml file
  - Start the navigation


How to set a new init pose:
  - In Rviz: 2D-Pose Estimate to approximately set the current pose of the robot
  - Drive through the arena to your desired inital pose (Check if robot is correctly localized by amcl)
  - `rosrun tf tf_echo /map /base_footprint` -> get the current pose of the robot in the map
  - in amcl.launch: change the three parameters "initial_pose_x", "initial_pose_y", "initial_pose_a" (orientation in rad)
  - Start the navigation
