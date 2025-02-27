#Kinematics

The first thing to understand is that this is not trivial.

Up to now, we've been relying upon the URDF and associated STL Meshes to allow us to visualize the appearance of a non-kinetic Swift Arm Pro.
As soon as we start moving the joints, we "disassemble" the robot.  Why would that be ?

The joint_state_publisher_gui is doing its job in relaying our manipulation of the slider control into the form of a message published onto the ros2 messaging system bus.  rViz is getting that message and doing what it's supposed to, as best it can, it it moving that joint.  But, **SOME** other joints stay in their last known positions.  It turns out, neither rViz or the joint_state_publsiher are designed to perform the kinematic transformtion of the **parallel** linked joints, and how those parallel linkages affect the other links. 

Not only that, but, since some of the joints (in fact the joints connecteing the "load bearing" linkages) don't have direct actuators, their motion relies upon the joints that do have them, these linkages are "driven" by other linkages, they are known as passive mechanical linkages.

So, we need to take care of that.  We need to perform both reverse, and forward kinematic computations for every joint state message.  The parallel and driven linkages need to be computed based upon their relations to the joint being manipulated.  Those calculations, by tradition, should then be published as a "Robot State" message.  It ins't entirely clear what the difference between Joint States and Robot State is - after all, the robot is a colleciton of joints, plus any end effector (also detailed in the URDF).