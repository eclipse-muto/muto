# Eclipse Muto VSCode Development Environment Setup with Podman and Dev Containers

This guide provides step-by-step instructions for setting up a VSCode development environment for Eclipse Muto using development containers with Podman. Using Visual Studio Code and Docker Containers will enable you to run your favorite ROS 2 Distribution without the necessity to change your operating system or use a virtual machine. With this guide you can set up a podman container, which can be used for your future ROS 2 projects.

You can also review instructions from [Setup ROS 2 with VSCode and Docker ](https://docs.ros.org/en/kilted/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html) if you prefer docker. Most instructiosn here will still work.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [System Setup](#system-setup)
3. [VSCode Configuration](#vscode-configuration)
4. [Development Container Setup](#development-container-setup)
5. [Workspace Configuration](#workspace-configuration)
6. [Building and Running](#building-and-running)
7. [Development Workflow](#development-workflow)
8. [Debugging and Testing](#debugging-and-testing)
9. [Troubleshooting](#troubleshooting)
10. [Advanced Configuration](#advanced-configuration)

---

## Prerequisites

### Required Software

- **Podman or Docker**: Container runtime
- **VSCode**: Visual Studio Code editor

---


## Development Container Setup

### 1. Create Workspace Structure

```bash

# Clone the Muto repositories
git clone --recurse-submodules https://github.com/eclipse-muto/muto.git
git submodule update  --recursive --remote

# make sure the src/agent module is on the symphony branch if the above
git submodule status

# should return smt like:
+f35c770eade63c8590d541e0ba9fb2c81921ade0 src/agent (heads/symphony)
 cdf7935f1e607e3e8478c0c520e5079d5c8ebc61 src/composer (heads/main)
 9ddf239ca84d649e5ba502a33fec76a392adedf3 src/core (heads/main)

# Create workspace directories
mkdir -p config launch .vscode .devcontainer
```

### 2. Development Container Configuration

Create `.devcontainer/devcontainer.json`:

```json
{
    "name": "Eclipse Muto Development Container",
    "remoteUser": "muto",
    "build": {
        "dockerfile": "Dockerfile",
        "args": {
            "USERNAME": "muto"
        }
    },
    "workspaceFolder": "/home/ws",
    "workspaceMount": "source=${localWorkspaceFolder},target=/home/ws,type=bind",
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-vscode.cpptools",
                "ms-vscode.cpptools-themes",
                "twxs.cmake",
                "donjayamanne.python-extension-pack",
                "eamodio.gitlens",
                "Ranch-Hand-Robotics.rde-pack",  
                "ms-azuretools.vscode-containers",
                "vstirbu.vscode-mermaid-preview"             
            ]
        }
    },
    "containerEnv": {
        "DISPLAY": "unix:0",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "ROS_DOMAIN_ID": "42"
    },
    "runArgs": [
        "-e", "DISPLAY=${env:DISPLAY}"
    ],
    "mounts": [
       "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached",
       "source=${env:HOME}/.ssh,target=/home/muto/.ssh,type=bind,consistency=cached"
    ],
    "postCreateCommand": "rosdep update || true && chown -R $(whoami) /home/ws/ && rosdep install --from-paths /home/ws --ignore-src -r -y"
}
```

### 3. Dockerfile Configuration

Create `.devcontainer/Dockerfile`:

```dockerfile
FROM ros:humble
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip   python3-flake8 bc  python3-autopep8   ssh curl git ros-humble-image-transport ros-humble-cv-bridge ros-humble-apriltag-msgs ros-humble-tf2-ros ros-humble-sensor-msgs ros-humble-demo-nodes-cpp ros-humble-ackermann-msgs
ENV SHELL /bin/bash
# RUN pip3 install install setuptools==58.2.0 
RUN pip3 install requests coverage jsons
RUN echo "source /opt/ros/humble/setup.bash" >> /home/muto/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> /home/muto/.bashrc
# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
```

---

## Workspace Configuration

### 1. Launch Configuration

Create `.vscode/launch.json`:

```json
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2: Muto",
            "type": "ros2",
            "request": "launch",
            "target": "${workspaceFolder}/launch/muto.launch.py",
            "arguments": ["vehicle_namespace:=org.eclipse.muto.test", "vehicle_name:=test-robot-debug"],
        }


    ]
}
```

---

## Building and Running

### 1. Open in Dev Container

1. **Open VSCode and navigate to your workspace:**
   ```bash
   cd ~/muto
   code .
   ```

2. **Open in Container:**
   - Press `Ctrl+Shift+P` to open command palette
   - Type and select: `Dev Containers: Reopen in Container`
   - Wait for container to build (first time will take several minutes)

### 2. Build the Workspace

Once inside the container:

```bash
# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 3. Run the Muto System

```bash
# Launch the complete Muto system
ros2 launch launch/muto.launch.py vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug

# Or use VSCode launch configuration
# Press F5 and select "ROS2: Muto"
```

---

## Development Workflow


### 2. Building and Testing

```bash
# Quick build (from VSCode terminal)
colcon build --packages-select agent

# Run tests
colcon test --packages-select agent
colcon test-result --verbose

# Run specific test files
cd src/agent
python3 -m pytest test/test_commands.py -v
```

### 3. Debugging

1. **Set Breakpoints**: Click in editor margin or press `F9`
2. **Start Debugging**: Press `F5` and select appropriate configuration
3. **Use Debug Console**: Evaluate expressions and inspect variables
4. **Step Through Code**: Use `F10` (step over), `F11` (step into), `F12` (step out)

---

## Debugging and Testing

### 1. ROS 2 Node Debugging

```bash
# Debug individual nodes
ros2 run --prefix 'gdb --args' agent muto_agent

# Monitor topics
ros2 topic list
ros2 topic echo /muto/gateway_to_agent

# Check node information
ros2 node list
ros2 node info /muto/agent
```

### 2. Unit Testing

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select agent

# Run with coverage
cd src/agent
python3 -m pytest --cov=agent test/

# Run specific test class
python3 -m pytest test/test_commands.py::TestCommandsPlugin -v
```

### 3. Integration Testing

```bash
# Test MQTT gateway
ros2 service call /muto/rostopic_list muto_msgs/srv/CommandPlugin "{}"

# Test command execution
ros2 topic pub /muto/agent_to_command muto_msgs/msg/MutoAction "
{
  context: '',
  method: 'ros/topic',
  payload: '',
  meta: {response_topic: 'test', correlation_data: '123'}
}" --once
```

---

## Troubleshooting

### Build Issues

**Problem**: Dependency errors
```bash
# Update rosdep
sudo rosdep update

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Check for missing Python packages
pip3 install -r requirements.txt  # if exists
```

**Problem**: Python path issues
```bash
# Check Python path in container
echo $PYTHONPATH

# Source workspace properly
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 3. ROS 2 Communication Issues

**Problem**: Nodes cannot communicate
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Set consistent domain ID
export ROS_DOMAIN_ID=42

# Check network configuration
ros2 daemon stop
ros2 daemon start
```

### 4. VSCode Extension Issues

**Problem**: IntelliSense not working
```bash
# Reload VSCode window
# Ctrl+Shift+P -> "Developer: reload window"

# Rebuild IntelliSense database  
# Ctrl+Shift+P -> "C/C++: rescan workspace"

# Check Python interpreter
# Ctrl+Shift+P -> "Python: select interpreter"
```

---

## Advanced Configuration

### 1. Multi-Architecture Support

For ARM64 development:

```dockerfile
# Add to Dockerfile
FROM --platform=linux/arm64 ros:humble
# or
FROM --platform=linux/amd64 ros:humble
```

### 2. Custom Development Images

Create custom base image for faster container startup:

```dockerfile
# custom-muto-dev.Dockerfile
FROM ros:humble

# Install all common dependencies
RUN apt-get update && apt-get install -y \
    python3-pip python3-pytest python3-black \
    ros-humble-rqt ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install paho-mqtt requests pytest-mock

# Pre-configure ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
```

Build and use:
```bash
podman build -t muto-dev:latest -f custom-muto-dev.Dockerfile .
# Update devcontainer.json to use "image": "muto-dev:latest" instead of "dockerfile"
```


---

## Summary

This guide provides a comprehensive setup for Eclipse Muto development using VSCode and Podman dev containers. Key benefits include:

- **Consistent Environment**: Same development environment across different machines
- **Isolated Dependencies**: Container isolates ROS 2 and Muto dependencies
- **Full IDE Integration**: Complete VSCode integration with debugging, testing, and IntelliSense
- **GUI Support**: X11 forwarding for ROS 2 GUI tools (RVIZ, rqt)
- **Podman Security**: Rootless containers with better security than Docker

The setup enables efficient development, testing, and debugging of Eclipse Muto components while maintaining a clean host system.