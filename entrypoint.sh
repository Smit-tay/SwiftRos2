#!/bin/bash
# entrypoint.sh
# Copyright 2025 Jack Sidman Smith
# Licensed under the MIT License. See LICENSE in project root.
#
# Runtime entrypoint for the swiftros2 container.
# Sources ROS2 and the swiftros2 install overlay, then runs the node.

source /opt/ros/jazzy/setup.bash

if [ ! -f /opt/swiftros2/install/setup.bash ]; then
    echo "ERROR: /opt/swiftros2/install/setup.bash not found." >&2
    echo "  Have you run NF_06 (rsync) to deploy the built artifacts?" >&2
    exit 1
fi

source /opt/swiftros2/install/setup.bash

term_handler() {
    kill -TERM "${HARDWARE_PID}" "${KINEMATICS_PID}" 2>/dev/null
    wait "${HARDWARE_PID}" "${KINEMATICS_PID}"
}
trap term_handler TERM
trap term_handler INT

ros2 run swiftpro_hardware swiftpro_hardware \
    --ros-args -p port:=${SWIFTPRO_PORT} &
HARDWARE_PID=$!

ros2 run swiftpro_kinematics swiftpro_kinematics &
KINEMATICS_PID=$!

wait "${HARDWARE_PID}" "${KINEMATICS_PID}"
