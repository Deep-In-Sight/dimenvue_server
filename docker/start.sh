#!/bin/bash

# Start dimenvue_server container with proper configuration
# This script starts a container with X11 support, volume mounts, and matching user permissions
#
# Usage: ./start_dimenvue_server.sh [-d|--detach]
#   -d, --detach    Force detached mode even in interactive terminal

set -e

CONTAINER_NAME="dimenvue_server"
FORCE_DETACHED=false
IMAGE_NAME="dimenvue_server:jetson"

# Detect platform - this script only works on Jetson
if [ ! -f /etc/nv_tegra_release ]; then
    echo "Error: This script is for Jetson platform only."
    exit 1
fi

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--detach)
            FORCE_DETACHED=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [-d|--detach]"
            echo "  -d, --detach    Force detached mode even in interactive terminal"
            echo "  -h, --help      Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Get current user UID, GID, and username
USER_ID=$(id -u)
GROUP_ID=$(id -g)
USER_NAME=$(whoami)

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting DimenvuePro server container...${NC}"

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${YELLOW}Container '${CONTAINER_NAME}' already exists.${NC}"

    # Check if it's running
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo -e "${YELLOW}Container is already running.${NC}"
        read -p "Do you want to attach to it? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            docker exec -it ${CONTAINER_NAME} bash
        fi
        exit 0
    else
        echo -e "${YELLOW}Starting existing container...${NC}"
        docker start ${CONTAINER_NAME}
        docker exec -it ${CONTAINER_NAME} bash
        exit 0
    fi
fi

# Check if image exists
if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
    echo -e "${RED}Error: Image '${IMAGE_NAME}' not found!${NC}"
    echo "Please build the image first: docker build --network=host -f docker/Dockerfile.jetson -t ${IMAGE_NAME} docker/"
    exit 1
fi

# Ensure X11 socket is accessible
xhost +local:root > /dev/null 2>&1 || true

echo -e "${GREEN}Creating new container with:${NC}"
echo "  - Image: ${IMAGE_NAME}"
echo "  - Name: ${CONTAINER_NAME}"
echo "  - User: ${USER_ID}:${GROUP_ID}"
echo "  - Network: host"
echo "  - X11: Enabled"
echo "  - Mounts:"
echo "      /f/shared_data -> /shared_data"
echo "      ~/dmv_data -> /dmv_data"
echo "      /media -> /media (rshared)"

# Detect if running in interactive terminal
if [ "$FORCE_DETACHED" = true ]; then
    # Forced detached mode
    echo -e "${GREEN}Running in detached mode (forced)...${NC}"
    DOCKER_FLAGS="-d"
    CMD="tail -f /dev/null"
elif [ -t 0 ]; then
    # Interactive mode
    echo -e "${GREEN}Running in interactive mode...${NC}"
    DOCKER_FLAGS="-it"
    CMD="bash"
else
    # Detached mode (no TTY)
    echo -e "${GREEN}Running in detached mode (no TTY detected)...${NC}"
    DOCKER_FLAGS="-d"
    CMD="tail -f /dev/null"
fi

# Start the container
docker run ${DOCKER_FLAGS} \
    --name ${CONTAINER_NAME} \
    --runtime nvidia \
    --network host \
    --privileged \
    -e LOCAL_UID=${USER_ID} \
    -e LOCAL_GID=${GROUP_ID} \
    -e LOCAL_USER=${USER_NAME} \
    -e DISPLAY=${DISPLAY} \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-user \
    -e LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:\${LD_LIBRARY_PATH} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /f/shared_data:/shared_data \
    -v ${HOME}/dmv_data:/dmv_data \
    --mount type=bind,source=/media,target=/media,bind-propagation=rshared \
    -v /run/udev:/run/udev:ro \
    ${IMAGE_NAME} \
    ${CMD}

# Cleanup X11 permissions on exit
xhost -local:root > /dev/null 2>&1 || true

if [ -t 0 ]; then
    echo -e "${GREEN}Container stopped.${NC}"
else
    echo -e "${GREEN}Container '${CONTAINER_NAME}' started successfully in detached mode.${NC}"
    echo -e "${YELLOW}To attach to it, run: ./attach_dimenvue.sh${NC}"
    echo -e "${YELLOW}  or: docker exec -it -u ${USER_NAME} ${CONTAINER_NAME} bash${NC}"
    echo -e "${YELLOW}To stop it, run: docker stop ${CONTAINER_NAME}${NC}"
fi
