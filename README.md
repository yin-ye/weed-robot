# weed-robot

A weeding robot built with rospy and simulated on Gazebo.

The thorvald robot navigates autonomously around the field using a predefined map. The robot moves between configured waypoints gotten via the Odometry topic. YOLOV2-TINY is used for the perception aspect, due to the speed of the network, and identifies and classifies the weeds into categories: baby_lettuce, mature_lettuce and onions. Once a weed is identified, the position of the weed is converted to world coordinates by using the camera model pinhole method and transforming the result of this to a real-time frame. The robot then moves to the position of the weed via move_base and sprays the weed.
