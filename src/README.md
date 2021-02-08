#####   Focus Area: Perception

The thorvald robot navigates autonomously around the field using a predefined map. The robot moves between configured waypoints gotten via the Odometry topic. YOLOV2-TINY is used for the perception aspect, due to the speed of the network, and identifies and classifies the weeds into categories: baby_lettuce, mature_lettuce and onions. Once a weed is identified, the position of the weed is converted to world coordinates by using the camera model pinhole method and transforming the result of this to a real-time frame. The robot then moves to the position of the weed via move_base and sprays the weed. 


#####   **HOW TO REPLICATE THE SOLUTION**
1.  Run the Thorvald simulation: `roslaunch weed_robot weed_robot.launch`

2.  Run the Darknet simulation: `roslaunch darknet_ros darknet_ros.launch`

3.  Run the Topological Navigation and Spraying nodes: `roslaunch weed_robot topnav_spray.launch`

4.  *Optional(used to view the map waypoints): `rviz -d $(rospack find uol_cmp9767m_tutorial)/config/topo_nav.rviz`*

