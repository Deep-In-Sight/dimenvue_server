#!/bin/bash
# Run a command inside the container with ROS2 environment sourced
#
# Usage: ./docker/run.sh <command> [args...]
# Example: ./docker/run.sh python3 -m pytest tests/ee/test_ee.py -v

set -e

CONTAINER_NAME="dimenvue_server"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running"
    echo "Start it with: ./docker/start.sh"
    exit 1
fi

# Build the command to run inside container
# Source ROS2 setup files then execute the provided command
docker exec -u linh "${CONTAINER_NAME}" bash -c "
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
$*
"
