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

SwiftRos2 is self-contained. It communicates with other ROS2 nodes via DDS over the
host network. No shared filesystem or shared container is required.

---

## Dependencies

### ROS2 packages
Standard ROS2 Jazzy packages — installed in the container images:
`rclcpp`, `rclcpp_action`, `sensor_msgs`, `geometry_msgs`, `std_msgs`

### libuarm (vendored, non-ROS)
The hardware node depends on [libuarm](https://github.com/Smit-tay/libuarm) — a
standalone C++ library for controlling the UArm Swift Pro over USB serial.

libuarm is **not** a ROS2 package and is not installed system-wide. It is vendored
under `third_party/libuarm/` and built as part of the SwiftRos2 workspace via
`add_subdirectory()`.

**One-time setup:**
```bash
mkdir -p third_party
cd third_party
git clone https://github.com/Smit-tay/libuarm.git
```

`third_party/` is excluded from the SwiftRos2 git repository (`.gitignore`).

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

**Note:** Only Joint1, Joint2, and Joint3 are published — the three physical servos.
Joint4 through Joint8 are mechanically constrained by the parallel linkage and
computed automatically by `robot_state_publisher` via `<mimic>` tags in the URDF.

---

### `swiftpro_kinematics`

Analytical forward kinematics node. Subscribes to `/joint_states` and computes
the Cartesian XYZ position of the end effector from first principles, publishing
the result as a `geometry_msgs/PointStamped` on `/swiftpro/end_effector_position`.

Does not broadcast TF — that is `robot_state_publisher`'s job. Provides a clean
Cartesian position topic for consumers that do not want to query TF.

**Publishers:**

| Topic | Type | Description |
|---|---|---|
| `/swiftpro/end_effector_position` | `geometry_msgs/PointStamped` | Analytical FK result in base frame |

**FK approach:** Geometric forward kinematics. Shoulder (Joint2) and elbow (Joint3)
both rotate around Y. End effector position is computed by projecting upper arm and
forearm lengths through the accumulated joint angles, then rotating around Z by the
base angle (Joint1). The parallel linkage keeps the end effector level and does not
affect tip position, so FK is correct from Joint1/2/3 alone.

---

### `swiftpro_resources`

Not a node — a ROS2 package containing shared resources:

- **Messages, services, actions:** `MoveToPosition.msg`, `MoveTo.srv`,
  `MoveToPosition.srv`, `SetGripper.srv`, `SetPump.srv`, `Reset.srv`,
  `MoveArm.action`
- **URDF/xacro:** `swiftpro.xacro` — full robot description with parallel linkage
  modelled via `<mimic>` joints
- **Launch files:** RViz2 visualisation launch
- **RViz2 configuration**

---

## Parallel Linkage Kinematics

The UArm Swift Pro uses two four-bar parallel linkages which URDF cannot represent
as closed kinematic chains (URDF is a tree structure). The solution is `<mimic>`
joints. `robot_state_publisher` applies these constraints automatically when given
Joint1/2/3 from the hardware node.

**Physical verification of constraints (observed on hardware):**

- Moving the shoulder (Joint2) only: forearm links stay parallel ✓
- Moving the elbow (Joint3) only: upper arm links stay parallel ✓

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

SwiftRos2 uses two separate container images. The development image is large (build
tools, IDE support, visualisation) and lives only on the NUC. The runtime image is
lean and lives on the worker.

| Concern | Development image | Runtime image |
|---|---|---|
| Size | ~3GB | ~300MB |
| Contents | Full toolchain, clangd, RViz2, colcon | Runtime dependencies only |
| Rebuilt when | Dockerfile changes | Dockerfile.runtime changes |
| Lives on | NUC | Worker |
| Source code | Mounted from host | Not present |
| Binaries | Built inside container, stored on host | Rsynced from NUC, mounted at runtime |

Neither image bakes SwiftRos2 binaries in. A code change requires only
`colcon build` + `rsync` — no image rebuild.

### Development image (`Dockerfile`)

Used on the NUC for building and development. Contains the full ROS2 Jazzy
toolchain, clangd, RViz2, and all build dependencies. The SwiftRos2 repository
(including `third_party/`) is mounted into the container at `/swiftros_ws`.

### Runtime image (`Dockerfile.runtime`)

Used on the worker. Based on `ros:jazzy-ros-core`. Contains only what
`swiftpro_hardware` needs to execute. Built artifacts are rsynced from the NUC to
`/opt/swiftpro/install` on the worker and mounted into the container at runtime.

---

## Registry

A private OCI registry runs on the worker as a systemd quadlet service.

```bash
sudo systemctl status registry
sudo systemctl restart registry
```

Registry data is persisted at `/opt/registry` (SELinux: `container_file_t`).
The NUC pushes to `worker:5000`; the worker pulls from `localhost:5000`.
Plain HTTP — acceptable for a private LAN.

Add to `~/.config/containers/registries.conf` on the NUC:

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
├── docker-compose.dev.yml        # Dev container configuration (NUC)
├── docker-compose.runtime.yml    # Runtime container configuration (worker)
├── entrypoint.sh                 # Runtime startup script — rsynced to worker
├── README.md                     # This file
├── DEPLOYMENT.md                 # Step-by-step deployment guide
├── third_party/                  # Vendored non-ROS dependencies (not in git)
│   └── libuarm/                  # UArm Swift Pro C++ library
├── swiftpro_hardware/
│   ├── CMakeLists.txt
│   ├── package.xml
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
# On the NUC — clone libuarm into third_party/
cd /home/jack/dev/smithjack.net/SwiftRos2
mkdir -p third_party && cd third_party
git clone https://github.com/Smit-tay/libuarm.git

# Build the development image
cd ..
podman build --format docker -f Dockerfile -t swiftros2_dev .

# Start the development container
podman-compose -f docker-compose.dev.yml up -d

# On the worker — ensure registry is running
systemctl status registry

# Build and push the runtime image (NF_08)
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

Full image rebuild (NF_08) only when system-level dependencies change.

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
# Start via compose (after NF_06 + NF_07, or manually)
cd /opt/swiftpro
podman-compose -f docker-compose.runtime.yml up -d

# Check it is running
podman ps | grep swiftpro_hardware
podman logs swiftpro_hardware
```

---

## Known Issues

- **Podman SHELL warning during image build:** `SHELL` is not supported in OCI image
  format. The warning is cosmetic. Root cause: `podman-compose` 1.5.0 does not pass
  `--format docker` to `podman build`. Workaround: build images directly with
  `podman build --format docker` (NF_08).

- **`ulimits` not permitted on NUC:** The dev compose omits `ulimits` because the
  NUC kernel does not permit `RLIMIT_RTPRIO` for non-privileged containers. The
  runtime compose on the worker retains them — the hardware node benefits from
  real-time scheduling priority.

- **`move_to` returns `error_code: -2`:** The arm moves correctly but the service
  reports failure. This is a return code handling issue in `swiftpro_hardware` —
  the uarm API `set_position` return value is not being interpreted correctly.
  Under investigation.

---

## Host Environment Requirements

### Rootless Podman — `userns_mode: keep-id` on Fedora 43

The development container uses `userns_mode: keep-id` so that build artifacts are
owned by the host user rather than root. On Fedora 43 with Podman 5.x,
`--userns=keep-id` hangs at container start unless `fuse-overlayfs` is explicitly
configured as the storage mount program.

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

Verify `fuse-overlayfs` is installed:
```bash
which fuse-overlayfs       # expected: /usr/bin/fuse-overlayfs
sudo dnf install fuse-overlayfs   # if missing
```

---

## Hardware Notes

- **Connection:** USB serial `/dev/ttyACM0` at 115200 baud
- **Firmware:** V4.9.1-jss.5 (custom fork of UFactory V4.9.1)
- **Servos:** 3 driven servos (base rotation, shoulder, elbow)
- **Parallel linkage:** Two four-bar loops keep the end effector level regardless
  of arm elevation. Modelled in URDF via `<mimic>` joints.
- **Coordinate system:** X forward, Y left, Z up. Units: millimetres for position
  commands (uarm API), metres for ROS2 topics.
- **Safe home position:** `x=200mm, y=0mm, z=150mm` — used by the Reset service.
- **Teach mode:** Servos can be detached via `M2019` (all) or `M2202 N<id>` (single)
  allowing the arm to be moved by hand while encoders continue reporting position.
  Re-attach with `M17`. See libuarm examples for a complete teach/record workflow.
