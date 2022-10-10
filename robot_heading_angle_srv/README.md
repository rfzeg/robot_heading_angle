# The robot_heading_angle_srv package

Author: Roberto Zegers R. 

ROS Distro: Galactic

**Disclaimer**:Limitation: at current, this only works when the robot's heading is 0 rad in the world reference frame. 

Assumptions:
- A robot exists: `odom` TF frame is broadcasted and `cmd_vel` topic exists.

Run it with:  
```
ros2 run robot_heading_angle_srv robot_heading_angle_node
```

To-Dos:  
-  Trigger the rotate functionallity by a service call, not a timer callback function.   