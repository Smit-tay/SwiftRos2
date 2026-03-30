# SwiftRos2 Deployment Guide

Here is the setup used by the developer of this project.  
This can be used as a reference for your own configuration.  

## Architecture
 
 Machines: 
- **NUC** (`192.168.179.38`) — development machine, builds the project
- **worker** (`192.168.179.25`) — deployment machine, physically connected to UArm Swift Pro via USB

SwiftRos2 runs in a lean runtime Podman container on worker. The container image
contains only the ROS2 runtime environment — project binaries are synced separately
via rsync and mounted as a volume at runtime. This means code changes only require
a rsync, not a full image rebuild.

---

## Prerequisites

### Both machines
ROS2 relies upon mDNS for topic discovery.  On Linux mDNS services are provided by  avahi-daemon which must be running on both NUC and worker for ROS2 cross-machine
discovery to work. Without it, nodes on different machines cannot find each other.
```bash
sudo dnf install -y avahi        # Fedora
sudo apt-get install -y avahi-daemon  # Ubuntu/Debian
sudo systemctl enable --now avahi-daemon
```

### worker only
Podman must be installed:
```bash
sudo dnf install -y podman
```

USB device permissions for the UArm Swift Pro:
```bash
sudo chmod a+rw /dev/ttyACM0
```

For persistent permissions across reboots, create a udev rule:
```bash
sudo bash -c 'echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"2341\", ATTRS{idProduct}==\"0042\", MODE=\"0666\"" > /etc/udev/rules.d/99-uarm.rules'
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## Initial Setup on worker

Run once to create the directory for project binaries:
```bash
sudo mkdir -p /opt/swiftros2/install
sudo chown -R <USER>:<GROUP> /opt/swiftros2
```

---

## Build (on NUC)
We provide development environment files for several IDEs.  The default is "geany" so, the appropriaet project and workspace files are present in the project root.  Other IDE configuration files may be found in the IDE directory.
```bash
# Build the project
cd /home/<USER>/dev/smithjack.net/SwiftRos2
podman exec topicfs bash -c "cd /home/<USER>/dev/smithjack.net/SwiftRos2 && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

# Sync binaries to worker (excludes STL mesh files)
rsync -av --exclude='*.STL' \
    /home/<USER>/dev/smithjack.net/SwiftRos2/install/ \
    worker:/opt/swiftros2/install/
```

Both commands are available as Geany build commands:
- **Build SwiftRos2** — builds the project
- **Sync SwiftRos2 to worker** — syncs binaries to worker

---

## Runtime Image (on NUC)

The runtime **container** image *only* needs to be rebuilt when system-level dependencies
change (new ROS2 packages, OS updates). It does NOT need rebuilding for code changes.
```bash
cd /home/<USER>/dev/smithjack.net/SwiftRos2
podman build -f Dockerfile.runtime -t swiftros2_runtime:latest .
```

### Transfer to worker

The following command streams the image directly from NUC to worker without
writing any temporary files to disk:
```bash
podman save swiftros2_runtime:latest | ssh worker "podman load"
```

`podman save` serializes the image to a tar stream on stdout. The pipe sends it
directly to `podman load` on worker via SSH. The image flows:
```
NUC podman store → tar stream → SSH tunnel → worker podman store
```

### Expected transfer output
```
Getting image source signatures
Copying blob sha256:...
[... one line per image layer ...]
Copying config sha256:...
Writing manifest to image destination
Loaded image: localhost/swiftros2_runtime:latest
```

---

## Run on worker
```bash
podman run --rm -it \
    --name swiftros2_runtime \
    --privileged \
    --network host \
    --userns=host \
    -v /dev:/dev \
    --security-opt label=disable \
    -e ROS_DOMAIN_ID=11 \
    -v /opt/swiftros2/install:/opt/swiftros2/install \
    swiftros2_runtime:latest
```

### Expected output on successful connection
```
connect success
recv thread start
[INFO] [...] [swiftros2_runtime]: Connected to UArm Swift Pro on /dev/ttyACM0
[INFO] [...] [swiftros2_runtime]: Device type: SwiftPro
[INFO] [...] [swiftros2_runtime]: Hardware version: 3.3.1
[INFO] [...] [swiftros2_runtime]: Firmware version: 3.2.0
[INFO] [...] [swiftros2_runtime]: SwiftRos2 "swiftpro_hardware" ROS2 node started - port /dev/ttyACM0
```

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| port | /dev/ttyACM0 | USB serial port |
| baudrate | 115200 | Serial baudrate |
| position_report_interval | 0.1 | Position report interval in seconds |
| joint_state_publish_rate | 10.0 | Joint state publish rate in Hz |

---

## Verifying the Running Node

Open a second terminal (worker) and exec into the running container:
```bash
# List active ROS2 topics
podman exec -it swiftros2_runtime bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /opt/swiftros2/install/setup.bash && \
     ros2 topic list"
```

Expected output:
```
/joint_states
/parameter_events
/rosout
/swiftros2/connected
/swiftros2/position
/swiftros2/pump_status
```

To inspect live data:
```bash
podman exec -it swiftros2_runtime bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /opt/swiftros2/install/setup.bash && \
     ros2 topic echo /swiftros2/connected"
```

---

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| /joint_states | sensor_msgs/JointState | Arm joint angles in radians |
| /swiftros2/position | geometry_msgs/Point | Arm XYZ position in mm |
| /swiftros2/pump_status | std_msgs/Bool | Suction pump on/off state |
| /swiftros2/connected | std_msgs/Bool | Arm connection state |

## Services

| Service | Type | Description |
|---------|------|-------------|
| /swiftros2/move_to | swiftros2_resources/MoveTo | Move arm to XYZ position |
| /swiftros2/set_pump | swiftros2_resources/SetPump | Control suction pump |
| /swiftros2/set_gripper | swiftros2_resources/SetGripper | Control gripper |
| /swiftros2/reset | swiftros2_resources/Reset | Reset arm to home position |
