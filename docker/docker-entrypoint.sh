#!/bin/bash
# Docker entrypoint script to create user with matching UID/GID

set -e

# Get user and group IDs from environment or use defaults
USER_ID=${LOCAL_UID:-1000}
GROUP_ID=${LOCAL_GID:-1000}
USERNAME=${LOCAL_USER:-dimenvue}

# Create group if it doesn't exist
if ! getent group $GROUP_ID >/dev/null; then
    groupadd -g $GROUP_ID $USERNAME
fi

# Create user if it doesn't exist
if ! getent passwd $USER_ID >/dev/null; then
    useradd -u $USER_ID -g $GROUP_ID -d /home/$USERNAME -m -s /bin/bash $USERNAME

    # Set up minimal bashrc
    cat > /home/$USERNAME/.bashrc << 'EOF'
# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace if it exists
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Set prompt
export PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
EOF

    chown $USER_ID:$GROUP_ID /home/$USERNAME/.bashrc
fi

# Switch to the user and execute the command
exec gosu $USER_ID:$GROUP_ID "$@"
