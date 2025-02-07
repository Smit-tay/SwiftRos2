# SwiftRos2  
ROS 2 Exploration with a UArm Swift Pro  by [UFACTORY](https://www.ufactory.cc/)

#### Table of Contents
- [About](#about)
  - [Architecture](#rchitecture)
- [Caution](#caution)
- [Setup](#setup)
  - [Step 1: Use the F***ing Container](#step-1-use-the-fing-container)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Basics](#test-the-basics)

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


## Setup

### Step 1: Use the F***ing Container  

Before doing anything else, **build the container**. If you’re not using the container, you’re making your life unnecessarily difficult. **Seriously, use it.**  

Why? Because ROS 2 is a sprawling mess of interdependent libraries, and installing everything directly on your machine is a recipe for polluting your system beyond repair. **Running ROS 2 outside of a container is a bad idea.** Full stop.  

Luckily, there’s an easy solution: Containers - and we're using **Docker**. The project includes a well-structured container setup with:  
- A **Dockerfile** (found in `src/Dockerfile`)  with a `src/docker-compose.yml` to help you properly use the Docker image.  
- A **VS Code Dev Container** configuration (`src/.devcontainer/devcontainer.json`) for painless use of the Container within VSCode.

This setup allows you to develop in an isolated, controlled environment with all dependencies preconfigured. **Use it, or enjoy debugging dependency hell.**  

For reference, this container approach is inspired by the genius work found here: [ssilenzi/devcontainer-humble-ros-base](https://github.com/ssilenzi/devcontainer-humble-ros-base).  

## Docker Container Installation
Please see this outstanding reference, for a general understandng of how to set up a Docker container.
I am sure you can translate those instructions to this project quite easily.

[Franka Robotics - franks_ros2/README.md](https://github.com/frankaemika/franka_ros2?tab=readme-ov-file#docker-container-installation)

# Build the project
Once your environment is set up properly (probably within a running container), you will need to perform a build.
This is quite simple, using the following commands:

```
# Source the ros environment
source /opt/ros/humble/setup.bash
```

```
# build the project
colcon build --symlink-install
```
```
# source the project specific environment
source install/setup.bash
```

# Test the Basics
# Visualizing the UArm Swift Pro in RViz  

Before doing anything else, let’s make sure we can **see** the robot in a visualizer.  

For **ROS 2**, the default visualization tool is **RViz**. Fortunately, we don’t even need to run any of our own nodes to get started. The goal here is simple:  
- Verify that RViz can render a graphical representation of the **UArm Swift Pro**.  

UArm has already provided a **Xacro file** on their website, which I have copied into my source tree along with corresponding **mesh files** (also provided by UArm) to define the robot’s visual model.  

## Xacro to URDF Conversion  

Because writing raw **URDF** would apparently be too straightforward, ROS has **Xacro**—an XML macro language designed to make URDF more readable (or arguably, more convoluted). Here’s what happens:  

- **Xacro (XML Macros)** allow you to avoid repeating yourself while making the XML even harder to manually debug.  
- We use a **Xacro file** to construct a [URDF file](http://wiki.ros.org/urdf/XML/model) that describes the robot’s structure:  
  - **Links**: Define rigid bodies with mass, visual features, and collision properties.  
  - **Joints**: Define how links connect and move relative to each other.  
- The **Xacro file** (`src/remote_node/urdf/swiftpro.xacro`) references multiple [STL files](https://en.wikipedia.org/wiki/STL_(file_format)) for precise shape descriptions.  
- The `xacro` command processes all these references and **generates a complete URDF file**.  
- This happens automatically at build time via CMake (`src/remote_node/CMakeLists.txt`).  

At the end of all this, we get a **URDF file**, which is basically just the fully expanded version of the **Xacro file**—a process that *definitely* doesn’t add unnecessary complexity (except when it does).  

This modularity is great for tweaking shapes, colors, or adding dynamic elements—but let’s be real: **it’s also Yet Another Failure Point (YAFP)** in this already over-engineered ecosystem.  

## Now That We Have a URDF, Let’s Actually Use It  

We provide a **ROS 2 launch file**:  `src/remote_node/launch/view_robot_urdf.launch.py`

Because manually starting every single ROS 2 component one by one would be far too annoying, **launch files** help orchestrate everything into a single command.  

### What This Launch File Does  

  - **Starts `joint_state_publisher_gui`**  

     _Launches the joint_state_publisher node:_ The GUI node starts the `joint_state_publisher` in the background, which is responsible for reading joint states and publishing them to the /joint_states topic.  

     _Manual joint control in RViz:_ The GUI allows you to manually move the robot’s joints in the RViz visualization, providing an interactive way to test joint positions and visualize the robot’s movements without needing external controllers or scripts.

   - **Starts `robot_state_publisher`**  

     _Publishes robot_description:_ This node takes the URDF (Unified Robot Description Format) or Xacro file and loads it as a ROS 2 parameter. It then publishes this information as the robot_description topic, which is essential for describing the robot’s structure and kinematics.  

     _Necessity of this node:_ Although it might seem like an extra step, this node is necessary because it handles the processing of the robot’s description and transforms it into a format that can be used by other parts of the system, like robot_state_publisher and RViz.

   - **Launches `RViz` with a predefined config**  `src/remote_node/config/swiftpro.rviz`

     _Loading the RobotModel plugin:_ The configuration specifies that the Robot Model plugin is loaded, subscribes to the /joint_states topic from joint_state_publisher and /robot_description from robot_state_publisher, then updates the visualization of the robot based on these inputs.  

     _Visualization and interaction:_ The plugin allows RViz to display the robot’s joint states, so you can visually see how the robot’s joints move when interacting with it in RViz, providing a real-time feedback loop for testing and simulation.

### Running the Launch File  

```bash
ros2 launch remote_node view_robot_urdf.launch.py
```
