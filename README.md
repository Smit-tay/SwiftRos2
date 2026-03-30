# SwiftRos2

A complete ROS2 driver stack for the UFactory UArm Swift Pro robotic arm.

**Author:** Jack Sidman Smith  
**ROS2 Distro:** Jazzy Jalisco  
**Platform:** Ubuntu 24.04 (containerised), Fedora 43 (host)  
**License:** MIT

---

## Overview

SwiftRos2 provides full ROS2 integration for the UArm Swift Pro — a 3-DOF desktop
robotic arm with a parallel linkage mechanism. It exposes the arm's capabilities as
standard ROS2 topics and services, making it compatible with any ROS2 consumer
including TopicFS, RViz2, and custom applications.

SwiftRos2 is designed to be self-contained. It communicates with other ROS2 nodes
(including TopicFS) via DDS over the host network. No shared filesystem or shared
container is required.

---

## Nodes

### `swiftpro_hardware`

The primary hardware interface node. Connects to the arm via USB serial
(`/dev/ttyACM0` at 115200 baud), publishes joint states and position data,
and exposes arm control via ROS2 services.

**Publishers:**

| Topic | Type | Description |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Joint1, Joint2, Joint3 servo angles in radians |
| `/swiftpro/position` | `geometry_msgs/Point` | Raw XYZ position from arm firmware |
| `/swiftpro/pump_status` | `std_msgs/Bool` | Vacuum pump on/off state |
| `/swiftpro/connected` | `std_msgs/Bool` | Arm connection state, updated every 5s |

**Services:**

| Service | Type | Description |
|---|---|---|
| `/swiftpro/move_to` | `swiftpro_resources/MoveTo` | Move to XYZ position at given speed |
| `/swiftpro/set_pump` | `swiftpro_resources/SetPump` | Enable or disable vacuum pump |
| `/swiftpro/set_gripper` | `swiftpro_resources/SetGripper` | Open or close gripper |
| `/swiftpro/reset` | `swiftpro_resources/Reset` | Move arm to safe home position |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyACM0` | USB serial port |
| `baudrate` | `115200` | Serial baud rate |
| `position_report_interval` | `0.1` | Arm firmware position report interval (seconds) |
| `joint_state_publish_rate` | `10.0` | JointState publish rate (Hz) |

**Important:** Only Joint1, Joint2, and Joint3 are published. These are the three
physical servos. Joint4 through Joint8 are mechanically constrained by the parallel
linkage and are computed automatically by `robot_state_publisher` via `<mimic>` tags
in the URDF. See the Kinematics section below.

---

### `swiftpro_kinematics`

Analytical forward kinematics node. Subscribes to `/joint_states` and computes
the Cartesian XYZ position of the end effector from first principles, publishing
the result as a `geometry_msgs/PointStamped` on `/swiftpro/end_effector_position`.

This node is complementary to `robot_state_publisher` — it does not broadcast TF
transforms (that is `robot_state_publisher`'s job). It provides a clean, readable
Cartesian position topic for consumers that do not want to query TF.

**Publishers:**

| Topic | Type | Description |
|---|---|---|
| `/swiftpro/end_effector_position` | `geometry_msgs/PointStamped` | Analytical FK result in Base frame |

**FK approach:** Geometric forward kinematics. The shoulder (Joint2) and elbow
(Joint3) both rotate around Y. The end effector position is computed by projecting
the upper arm and forearm lengths through the accumulated joint angles, then
rotating the result around Z by the base angle (Joint1). The parallel linkage
keeps the end effector level — it does not affect the tip position, only the
orientation, so FK is correct from Joint1/2/3 alone.

Cross-check: at all joints zero, the expected position is approximately
`x=0.156m, y=0.0m, z=0.248m`.

---

### `swiftpro_resources`

Not a node — a ROS2 package containing shared resources used by all other packages:

- **Messages, services, actions:** `MoveToPosition.msg`, `MoveTo.srv`,
  `MoveToPosition.srv`, `SetGripper.srv`, `SetPump.srv`, `Reset.srv`,
  `MoveArm.action`
- **URDF/xacro:** `swiftpro.xacro` — full robot description with parallel linkage
  modelled via `<mimic>` joints (see Kinematics section)
- **Launch files:** RViz2 visualisation launch
- **RViz2 configuration**

---

## Parallel Linkage Kinematics

The UArm Swift Pro uses two four-bar parallel linkages which URDF cannot represent
as closed kinematic chains (URDF is a tree structure). The solution is to model the
constrained joints as `<mimic>` joints in the URDF. `robot_state_publisher` applies
these constraints automatically when given Joint1/2/3 from the hardware node.

**Physical verification of constraints (observed on hardware):**

- Moving the shoulder (Joint2) only: forearm links stay parallel ✓
- Moving the elbow (Joint3) only: upper arm links stay parallel ✓

This confirms that shoulder movement is fully cancelled within the upper arm
parallelogram loop and never reaches the end effector. Only elbow rotation (Joint3)
propagates to the wrist.

**Mimic relationships:**

| Joint | Mimics | Ratio | Role |
|---|---|---|---|
| Joint4 | Joint3 | +1.0 | Triangular connector — transfers elbow rotation to wrist chain |
| Joint5 | Joint2 | -1.0 | Upper arm rear rod — closes upper arm parallelogram loop |
| Joint6 | Joint2 | +1.0 | Wrist rear upper rod — follows shoulder |
| Joint7 | Joint3 | +1.0 | Wrist rear lower rod — transfers elbow to wrist |
| Joint8 | Joint3 | -1.0 | End effector mount — cancels elbow rotation, keeps gripper level |
| Joint9 | fixed  | —    | Gripper jaw — not servo-driven in base configuration |

**Node graph for visualisation:**

```
swiftpro_hardware
    publishes → /joint_states  {Joint1, Joint2, Joint3}
                      ↓
         robot_state_publisher
         (reads URDF, applies <mimic> constraints for Joint4-8)
              publishes → /tf
              publishes → /robot_description
                      ↓
                   RViz2

swiftpro_kinematics
    subscribes → /joint_states
    publishes  → /swiftpro/end_effector_position
```

---

## Container Architecture

SwiftRos2 uses two separate container images with distinct purposes. This separation
is a deliberate design choice: the development image is large (build tools, IDE
support, visualisation) and lives only on the development machine (NUC). The runtime
image is lean and lives on the deployment target (worker).

### Why two images?

| Concern | Development image | Runtime image |
|---|---|---|
| Size | ~3GB — acceptable for a dev machine | ~300MB — appropriate for a headless server |
| Contents | Full toolchain, clangd, RViz2, colcon | Runtime dependencies only |
| Rebuilt when | Dockerfile changes | Dockerfile.runtime changes |
| Lives on | NUC (dev machine) | Worker (deployment target) |
| Source code | Mounted from host at runtime | Not present — binaries only |
| Binaries | Built inside container, stored on host | Rsynced from NUC, mounted at runtime |

Critically — **neither image bakes the SwiftRos2 binaries in**. The development
image mounts the entire source tree from the NUC host. The runtime image mounts the
built artifacts from the worker host. This means a code change never requires
rebuilding either image — only a `colcon build` and an `rsync`.

### Development image (`Dockerfile`)

Used on the NUC for building and development. Contains the full ROS2 Jazzy
development toolchain, clangd for IDE integration, RViz2 for visualisation, and
all SwiftRos2 build dependencies.

The entire SwiftRos2 repository is mounted into the container at `/swiftros_ws`.
`colcon build` outputs to `/swiftros_ws/build` and `/swiftros_ws/install`, both of
which are on the NUC host filesystem and survive container restarts.

### Runtime image (`Dockerfile.runtime`)

Used on the worker for running the hardware node in production. Based on
`ros:jazzy-ros-core` (minimal ROS2 runtime, no build tools). Contains only the
ROS2 packages that `swiftpro_hardware` needs to execute.

The built artifacts (`/swiftros_ws/install` on the NUC) are rsynced to
`/opt/swiftpro/install` on the worker and mounted into the runtime container.
The startup script (`entrypoint.sh`) is also rsynced and referenced directly
from the compose file — it is not baked into the image.

This means updating the running node on the worker requires only:
1. `colcon build` on the NUC (NF_04)
2. `rsync` to the worker (NF_06)
3. Restart the container on the worker (NF_07)

A full image rebuild (NF_08) is only needed when system-level dependencies change.

---

## Registry

A private OCI registry runs on the worker as a systemd service (via Podman quadlet).
This registry stores the runtime image and allows the NUC to push updates without
manually transferring image archives.

The registry is configured at `/etc/containers/systemd/registry.container` and
managed by systemd:

```bash
sudo systemctl status registry
sudo systemctl restart registry
```

Registry data is persisted at `/opt/registry` on the worker with SELinux context
`container_file_t`.

The NUC pushes to `worker:5000`. The worker pulls from `localhost:5000`.
The registry runs over plain HTTP (no TLS) — acceptable for a private LAN.

To allow the NUC to push to the insecure registry, add to
`~/.config/containers/registries.conf` on the NUC:

```toml
[[registry]]
location = "worker:5000"
insecure = true
```

---

## Directory Structure

```
SwiftRos2/
├── Dockerfile                    # Development image
├── Dockerfile.runtime            # Lean runtime image for worker deployment
├── docker-compose.yml            # Dev container configuration (NUC)
├── docker-compose.runtime.yml    # Runtime container configuration (worker)
├── entrypoint.sh                 # Runtime startup script — rsynced to worker
├── README.md                     # This file
├── DEPLOYMENT.md                 # Step-by-step deployment guide
├── swiftpro_hardware/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── lib/                      # Bundled UArm C++ serial library
│   └── src/
│       └── swiftpro_hardware.cpp
├── swiftpro_kinematics/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src/
│       └── swiftpro_kinematics.cpp
└── swiftpro_resources/
    ├── CMakeLists.txt
    ├── package.xml
    ├── messaging/
    │   ├── action/MoveArm.action
    │   ├── msg/MoveToPosition.msg
    │   └── srv/
    │       ├── MoveTo.srv
    │       ├── MoveToPosition.srv
    │       ├── Reset.srv
    │       ├── SetGripper.srv
    │       └── SetPump.srv
    ├── launch/
    │   └── view_robot_urdf.launch.py
    └── urdf/
        ├── swiftpro.xacro
        └── *.STL
```

---

## Development Workflow

### One-time setup

```bash
# On the NUC — build the development image
cd /home/<USER>/dev/SwiftRos2
# podman will complain about Docker's use of no OCI instructions - use env. var.
BUILDAH_FORMAT=docker \
  podman compose -f docker-compose.dev.yml build --no-cache --pull
# Start the development container
podman-compose up -d

# On the worker — ensure registry is running
systemctl status registry

# On the NUC — build and push the runtime image (NF_08)
podman build --format docker \
  -f Dockerfile.runtime \
  -t worker:5000/swiftpro_hardware:latest .
podman push --tls-verify=false worker:5000/swiftpro_hardware:latest

# On the worker — pull the runtime image
podman pull --tls-verify=false localhost:5000/swiftpro_hardware:latest
```

### Normal development cycle

```
Edit code on NUC
    → NF_04  colcon build in dev container
    → NF_06  rsync install/ + entrypoint.sh + docker-compose.runtime.yml to worker
    → NF_07  restart runtime container on worker
```

### IDE menu reference

| Entry | Action |
|---|---|
| SwiftRos2 - Build | colcon build inside dev container |
| SwiftRos2 - Clean | remove build/ install/ log/ |
| SwiftRos2 - Deploy to worker | rsync artifacts + scripts to worker |
| SwiftRos2 - Restart on worker | restart runtime container via SSH |
| SwiftRos2 - Build+Push runtime image | rebuild runtime image, push to registry |

---

## Running on the Worker

```bash
# Pull latest runtime image (after NF_08)
podman pull --tls-verify=false localhost:5000/swiftpro_hardware:latest

# Start via compose (after NF_06 + NF_07, or manually)
cd /opt/swiftpro
podman-compose -f docker-compose.runtime.yml up -d

# Check it is running
podman ps | grep swiftpro_hardware
podman logs swiftpro_hardware

# Verify topics are visible on the network
ros2 topic list
ros2 topic echo /joint_states
```

---

## Known Issues

- **Podman SHELL warning during image build:** `SHELL` is not supported in OCI image
  format. The warning is cosmetic — the image builds and runs correctly. Root cause:
  `podman-compose` 1.5.0 does not pass `--format docker` to `podman build`.
  Workaround: build images directly with `podman build --format docker` (NF_08).

- **`ulimits` not permitted on NUC:** The `docker-compose.yml` dev configuration
  omits `ulimits` because the NUC kernel does not permit `RLIMIT_RTPRIO` for
  non-privileged containers. The `docker-compose.runtime.yml` on the worker retains
  them because the hardware node benefits from real-time scheduling priority.

---

## Host Environment Requirements

### Rootless Podman — `userns_mode: keep-id` on Fedora 43

The development container uses `userns_mode: keep-id` so that files created inside
the container (build artifacts, log directories) are owned by your host user rather
than root. Without this, every `colcon build` creates root-owned files in the
mounted source tree, causing permission errors on subsequent builds.

On Fedora 43 with Podman 5.x, `--userns=keep-id` hangs at container start unless
`fuse-overlayfs` is explicitly configured as the storage mount program. The default
kernel overlay driver does not support idmapped mounts in this configuration.

**One-time fix — create `~/.config/containers/storage.conf`:**
```bash
mkdir -p ~/.config/containers
cat > ~/.config/containers/storage.conf << 'EOF'
[storage]
  driver = "overlay"

[storage.options.overlay]
  mount_program = "/usr/bin/fuse-overlayfs"
EOF
```

Verify `fuse-overlayfs` is installed first:
```bash
which fuse-overlayfs
# expected: /usr/bin/fuse-overlayfs
# if missing: sudo dnf install fuse-overlayfs
```

This setting is persistent and applies to all rootless podman containers on the
host. It has no effect on rootful podman (sudo podman). Sonnet 4.6Claude is

---

## Hardware Notes

- **Connection:** USB serial `/dev/ttyACM0` at 115200 baud
- **Servos:** 3 driven servos (base rotation, shoulder, elbow)
- **Parallel linkage:** Two four-bar loops keep the end effector level regardless
  of arm elevation. Modelled in URDF via `<mimic>` joints — no separate kinematics
  node required for visualisation.
- **Coordinate system:** X forward, Y left, Z up. Units: millimetres for position
  commands (UArm API), metres for ROS2 topics.
- **Safe home position:** `x=200mm, y=0mm, z=150mm` — used by the Reset service
  when no explicit position is provided.

## Firmware Version

SwiftRos2 was developed and tested against firmware 3.2.0. A newer series exists
(V4.0–V4.10.0) but was never accompanied by an updated C++ SDK or protocol
documentation — the V4.x changes were only reflected in a Python-only SDK. The one
potentially useful addition in V4.x is closed-loop step-loss correction, but the
dominant positional error in practice is mechanical gearbox backlash, which no
firmware can fix. A firmware upgrade would require rework for no meaningful gain,
so 3.2.0 is the tested and recommended version for this stack.
