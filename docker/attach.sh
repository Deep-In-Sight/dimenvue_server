#!/bin/bash
# Attach to the dimenvue_server container as the proper user

CONTAINER_NAME="dimenvue_server"
USER_NAME=$(whoami)

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it with: ./start_dimenvue_server.sh"
    exit 1
fi

# Attach as the user
docker exec -it -u ${USER_NAME} ${CONTAINER_NAME} bash
