#!/bin/bash
set -e

# Change to script directory
cd "$(dirname "${BASH_SOURCE[0]}")"

CONTAINER_NAME="dimenvue_server"

# Restart existing container if it exists
echo "Checking if container exists"
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Restarting existing container..."
    docker restart "${CONTAINER_NAME}" 2>/dev/null || true
else
    echo "Starting new container"
    ./start.sh -d
    sleep 1
fi

# Start uvicorn in detached mode with ROS environment sourced
echo "Starting API server"
docker exec -d -u "$(whoami)" "${CONTAINER_NAME}" bash -c "
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /opt/dimenvue_server && uvicorn server_app:app --host 0.0.0.0 --port 8000 > /tmp/dimenvue_server.log 2>&1
"

echo "Waiting for server to start..."
for i in {1..10}; do
    if curl -s http://localhost:8000/health > /dev/null 2>&1; then
        echo "Server is running at http://localhost:8000"
        exit 0
    fi
    sleep 1
done

echo "Warning: Server may not have started correctly. Check with: curl http://localhost:8000/health"
exit 1
