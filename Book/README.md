# SwiftRos2
ROS2 with a UArm Swift Pro by [UFACTORY](https://www.ufactory.cc/)

![UArm Swift Pro in RViz](Book/assets/images/InTheBeginning.png)

## Table of Contents
- [About](#about)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Get Started](#get-started)
- [Deployment](#deployment)
- [Network Requirements](#network-requirements)
- [Caution](#caution)

## About

This project provides a ROS2 hardware abstraction layer for the UArm Swift Pro —
a relatively affordable 4-degree-of-freedom robotic arm manufactured by UFACTORY.

The primary goal is a clean, simple ROS2 interface to the arm. Simple enough to
be educational, complete enough to be actually useful.

Much of this project can be explored without a physical robot. But to really
understand what's going on, a real robot helps. A lot.

The UArm Swift Pro is available pre-assembled. You could theoretically 3D print
the parts and build one from scratch — but let's be honest, just buy one.
It's faster, easier, and unless you genuinely enjoy frustration, a few shifts
at McDonald's will cover it.

## Architecture

Four ROS2 packages:

- **swiftpro_hardware** — communicates with the arm over USB serial. Publishes
  joint states, position, pump status, and connection state. Accepts move, pump,
  gripper, and reset service calls.

- **swiftpro_kinematics** — TF transform publisher. The UArm Swift Pro uses a
  parallel linkage mechanism that URDF cannot accurately describe. This node
  computes and publishes the correct transforms for all links based on reported
  joint angles. Work in progress.

- **swiftpro_resources** — URDF/xacro robot description, STL meshes, RViz
  configurations, launch files, and all custom message/service/action definitions.

- **swiftpro_vision** — placeholder for future camera-based tile detection.
  Not yet implemented.

The hardware node runs on a separate machine physically connected to the arm.
See [Deployment](#deployment).

## Prerequisites

- Ubuntu 24.04 (Noble) — the container base image
- ROS2 Jazzy Jalisco
- Podman and podman-compose
- avahi-daemon running on all machines (see [Network Requirements](#network-requirements))

## Get Started

[Getting Started](Book/GettingStarted.md) — build the project, visualize the
robot in RViz, understand why the parallel linkage breaks things.

[Kinematics Node](Book/KinematicsNode.md) — why URDF isn't enough and what
we do about it.

## Deployment

The hardware node runs on a separate machine physically connected to the arm
via USB. See [DEPLOYMENT.md](DEPLOYMENT.md) for full instructions.

## Network Requirements

ROS2 Jazzy uses Fast DDS as its default middleware, which relies on multicast
for automatic node discovery across machines on the same LAN.

**avahi-daemon must be running on all machines running ROS2 nodes.**

Without it, Fast DDS multicast discovery fails silently — nodes on different
machines cannot find each other even on the same subnet. This shows up as
`ros2 topic list` returning only local topics with nothing from remote machines.
```bash
# Fedora/RHEL
sudo dnf install -y avahi
sudo systemctl enable --now avahi-daemon

# Ubuntu/Debian
sudo apt-get install -y avahi-daemon
sudo systemctl enable --now avahi-daemon
```

Not required when all nodes run on the same machine.

## Caution

This package is under active development. Breaking changes should be expected.
Bug reports via the [GitHub Issues page](https://github.com/Smit-tay/SwiftRos2/issues)
are welcome.

## License

Apache-2.0. See [LICENSE](LICENSE).

## Author

Jack Sidman Smith — [smithjack.net](https://smithjack.net)
