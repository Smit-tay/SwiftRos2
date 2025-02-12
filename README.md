# SwiftRos2  
ROS 2 Exploration with a UArm Swift Pro  by [UFACTORY](https://www.ufactory.cc/)

#### Table of Contents
- [About](#about)
  - [Architecture](#architecture)
- [Caution](#caution)
- [Get Started !](#get-started)
  - [Getting Started](Book/GettingStarted.md)
  - [Kinematics Node](Book/KinematicsNode.md)

# About  
This project is designed to provide a fundamental understanding of **ROS 2's** core features and design paradigms.  Much of this project may be used without a physical robot.  BUT, to really "grok" what we're doing a real robot is helpful.

We have chosen the **UArm Swift Pro**, a relatively affordable, **4-degree-of-freedom robotic arm** manufactured by **UFACTORY**.  

Technically, you *could* 3D print the parts, assemble them, and integrate an **Arduino-compatible controller board** to build a fully functional **UArm Swift Pro** robotic arm from scratch. **But let’s be honest, just buy one pre-assembled.** It’s faster, easier, and unless you *really* enjoy frustration, you’re probably better off flipping burgers at McDonald’s for a few shifts and using the paycheck to buy the real deal.  


## Architecture  
- A **Low-Level Controller** (**controller_node**) that communicates directly with the UArm Swift Pro.  
- A **Remote Controller** (**remote_node**) that publishes commands via ROS 2 messaging to the **controller_node**.  
- The **remote_node** also receives robot state updates from the **controller_node**, enabling visualization in RViz.  

The main focus of this project is simplicity.  The BEST examples are the simplest example, which illustrate only the important concepts.  We will try to keep this philosphy in mind throughout this project.  BUT, future plans do include more advanced features.

# Caution
This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/Smit-tay/SwiftRos2/issues).

With that in mind, let's get going:  

# Get Started
[Getting Started](Book/GettingStarted.md)
