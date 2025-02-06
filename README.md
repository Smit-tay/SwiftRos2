# SwiftRos2  
ROS 2 Exploration with a UArm Swift Pro  by [UFACTORY](https://www.ufactory.cc/)

## Overview  

This project is designed to provide a fundamental understanding of **ROS 2's** core features and design paradigms.  

At the heart of this project is the **UArm Swift Pro**, a relatively affordable, **4-degree-of-freedom robotic arm** manufactured by **UFACTORY**.  

Technically, you *could* 3D print the parts, assemble them, and integrate an **Arduino-compatible controller board** to build a fully functional robotic arm from scratch. **But let’s be honest—just buy one pre-assembled.** It’s faster, easier, and unless you *really* enjoy frustration, you’re probably better off flipping burgers at McDonald’s for a few shifts and using the paycheck to buy the real deal.  


### Architecture  
- A **Low-Level Controller** (**controller_node**) that communicates directly with the UArm Swift Pro.  
- A **Remote Controller** (**remote_node**) that publishes commands via ROS 2 messaging to the **controller_node**.  
- The **remote_node** also receives robot state updates from the **controller_node**, enabling visualization in RViz.  

The initial goal is to keep things as simple as possible, but future plans include more advanced features like collision detection and beyond.  

## Getting Started  

### Step 1: Use the F***ing Container  

Before doing anything else, **build the container**. If you’re not using the container, you’re making your life unnecessarily difficult. **Seriously, use it.**  

Why? Because ROS 2 is a sprawling mess of interdependent libraries, and installing everything directly on your machine is a recipe for polluting your system beyond repair. **Running ROS 2 outside of a container is a bad idea.** Full stop.  

Luckily, there’s an easy solution: **Docker**. The project includes a well-structured container setup with:  
- A **Dockerfile** (found in `src/Dockerfile`).  
- A **VS Code Dev Container** configuration (`.devcontainer/` and `docker-compose.yml`).  

This setup allows you to develop in an isolated, controlled environment with all dependencies preconfigured. **Use it, or enjoy debugging dependency hell.**  

For reference, this container approach is inspired by the excellent work of [ssilenzi/devcontainer-humble-ros-base](https://github.com/ssilenzi/devcontainer-humble-ros-base).  

Now, build the container and get to work.

Basic commands:

```
# Source the ros environment
. /opt/ros/humble/setup.sh
```

```
# build the project
colcon build --symlink-install
```
```
# source the project specific environment
. install/setup.sh
```
# Visualizing the UArm Swift Pro in RViz  

## Step 1: Get the Robot on Screen  

Before doing anything else, let’s make sure we can **see** the robot in a visualizer.  

For ROS 2, the default visualization tool is **RViz**. Fortunately, we don’t even need to run any of our own nodes to get started. The goal here is simple:  
- Verify that RViz can render a graphical representation of the **UArm Swift Pro**.  

UArm has already provided a **Xacro file** on their website, which I have copied into my source tree along with corresponding **mesh files** (also provided by UArm) to define the robot’s visual model.  

### How It Works  

1. **Xacro to URDF Conversion**  
   - The **Xacro file** (`src/remote_node/urdf/swiftpro.xacro`) is converted into a **URDF file** using the `xacro` command.  
   - This is handled automatically via the **CMakeLists.txt** file in `src/remote_node/`.  

2. **STL Meshes**  
   - The Xacro file references **STL mesh files** in the same directory to define the robot’s shape.  
   - The `xacro` command processes these references and **generates a complete URDF file**.  

### Xacro vs. URDF  

At the core, a **URDF** file is just a fully expanded version of a **Xacro** file. The difference?  
- Xacro allows **modularity**, letting you define reusable components.  
- URDF is the final, **flattened** version with all STL references resolved.  

This modularity is useful for tweaking shapes, colors, or adding dynamic elements—but let’s be real: **it’s also Yet Another Failure Point (YAFP)** in this already complex ecosystem.  

Now - for the normal person - you will require 3 seperate terminals 

Terminal 1.

2024.FEB.07-00:39 UCT 
To Be COntinued