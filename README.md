# SwiftRos2
ROS2 exploration with a UArm SwiftPro 

This project is designed to enable a fundamental understanding some of the features and design paradigms of ROS2.

The design of this project is based around a Low Level controller (controller_node) which communicates with the actual Robot (in the case a UArm Swift Pro), as well as a remote controller (remote_node) that publishes commands (via ros2 messaging) to the controller_node.

The remote_node will also receive robot state via messages from the controller_node, and this should allow a visualization tool (rViz) to be used.

I am trying to keep this as simple as possible for now, but, eventually will add in more advanced features like collision detection, and more !
