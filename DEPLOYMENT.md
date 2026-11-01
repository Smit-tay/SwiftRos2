# SwiftRos2 Deployment Guide

This is the setup used by the developer of this project. Use it as a reference
for your own configuration.

## Architecture

- **NUC** (`192.168.179.38`): development machine. Builds everything in Podman
  containers.
- **worker** (`192.168.179.25`): deployment machine, connected to the UArm Swift
  Pro by USB. Runs `swiftros2_runtime`, `topicfs_runtime`, and Apache for
  `robot.smithjack.net`.

ROS2 is never installed on either host. The arm must run the
[SwiftPro-plusplus](https://git.smithjack.net/jack/SwiftPro-plusplus) firmware
(`5.0.0-pp.1` at the time of writing).

**Only one machine may run `swiftpro_hardware` on a given `ROS_DOMAIN_ID`.** Two
copies offer the same services under the same names, and a caller can reach
either one. The stack uses domain 11; use another domain for isolated tests.

The runtime image holds only the ROS2 runtime. Project binaries are rsynced to
`/opt/swiftros2` and mounted as a volume, so code changes need an rsync and a
restart, not an image rebuild.

Compose is the Go `docker-compose` binary talking to Podman. Do not use
`podman-compose`. Docker itself is not used: its service is disabled on worker.

---

## Prerequisites

### Both machines

Both machines must be on the same network and allow DDS discovery traffic (UDP
multicast) between them. Every DDS participant sets
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`; the compose files do this for you.

```bash
sudo dnf install -y podman docker-compose
```

### worker only

Podman's system socket, which `docker-compose` talks to:
```bash
sudo systemctl enable --now podman.socket
```

USB permissions for the arm, persistent across reboots:
```bash
sudo bash -c 'echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"2341\", ATTRS{idProduct}==\"0042\", MODE=\"0666\"" > /etc/udev/rules.d/99-uarm.rules'
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The image registry, `systemd-registry`, on port 5000.

The directory for project binaries:
```bash
sudo mkdir -p /opt/swiftros2/install
sudo chown -R <USER>:<GROUP> /opt/swiftros2
```

---

## Build (on NUC)

```bash
cd ~/dev/smithjack.net/SwiftRos2
docker-compose -f docker-compose.dev.yml up -d     # first time: starts swiftros2_dev

podman exec swiftros2_dev bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /swiftros_ws && \
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
```

Do not use `--symlink-install`: the install tree is rsynced to worker.

## Deploy (from NUC)

```bash
cd ~/dev/smithjack.net/SwiftRos2
rsync -av --exclude='*.STL' install/ worker:/opt/swiftros2/install/
rsync -av entrypoint.sh docker-compose.runtime.yml worker:/opt/swiftros2/
```

## Start or restart (on worker)

```bash
cd /opt/swiftros2
sudo env DOCKER_HOST=unix:///run/podman/podman.sock \
    docker-compose -f docker-compose.runtime.yml up -d --force-recreate
```

If this reports that the container name is already in use, the old container was
not created by compose. Remove it with `sudo podman rm -f swiftros2_runtime` and
run the command again. Its files live on the host, so nothing is lost.

## Runtime image (on NUC)

Rebuild only when system-level dependencies change, not for code changes.
```bash
cd ~/dev/smithjack.net/SwiftRos2
podman build --format docker -f Dockerfile.runtime \
    -t localhost:5000/swiftros2_runtime:latest .
podman push --tls-verify=false localhost:5000/swiftros2_runtime:latest \
    worker:5000/swiftros2_runtime:latest
```

---

## TopicFS and the website

`robot.smithjack.net` reaches the arm through TopicFS, in its own repository.
From the TopicFS repo on NUC, after building:
```bash
scripts/deploy_to_worker.sh
```
Then on worker:
```bash
/opt/topicfs/start.sh     # clears stale mounts, recreates the container
/opt/topicfs/stop.sh
```

`/mnt/topicfs` must be a shared bind mount. This line in worker's `/etc/fstab`
makes it one at boot:
```
/mnt/topicfs /mnt/topicfs none bind,shared,nofail 0 0
```

The compose file exposes only the services the website needs, through
`TOPICFS_ALLOWED_SERVICES`.

Website files are in the `smithjack-web` repository under `robot/`, deployed to
`/var/www/robot.smithjack.net`. Motion limits are in `robot/limits.json`. They
are taught by hand and stop being valid if the arm or its table moves.

Public motion has a kill switch on worker. The page and camera keep working:
```bash
sudo touch /var/www/robot_disabled    # motion off
sudo rm /var/www/robot_disabled       # motion on
```

Containers are not started automatically at boot, because
`podman-restart.service` is disabled on worker. Start them by hand after a reboot.

---

## Node parameters

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyACM0` | USB serial port |
| `baudrate` | `115200` | Serial baud rate |
| `position_report_interval` | `0.1` | Firmware position report interval, seconds |
| `status_poll_rate` | `1.0` | Status poll rate, Hz |
| `watchdog_period` | `5.0` | Reconnect and report watchdog period, seconds |
| `z_min` | `5.0` | Lowest Cartesian target, mm. Joint moves are not checked |

## Verifying

On worker:
```bash
sudo podman ps
sudo podman logs --tail 20 swiftros2_runtime
```

A good start ends like this:
```
[swiftpro_hardware]: Firmware version: 5.0.0-pp.1
[swiftpro_hardware]: encoders: all healthy
[swiftpro_hardware]: connect: ready on /dev/ttyACM0
[swiftpro_hardware]: swiftpro_hardware ready — port=/dev/ttyACM0 baud=115200 ...
```

Check for exactly one publisher:
```bash
sudo podman exec swiftros2_runtime bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /opt/swiftros2/install/setup.bash && \
     ros2 topic info /swiftpro/connected"
```

Through TopicFS and the website:
```bash
cat /mnt/topicfs/swiftpro/connected/latest
curl https://robot.smithjack.net/status.php
```

The full test, `test_swiftros2.sh`, moves the arm, so clear the bench first. It
expects `swiftpro_hardware` to be running in the same domain. See `README.md` for
the topics and services, speed units and error codes.
