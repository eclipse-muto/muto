# Quick Start with Eclipse Muto Containers and Eclipse Symphony

## Introduction

This guide demonstrates how to set up and deploy Eclipse Muto using pre-built container images with Eclipse Symphony integration for distributed ROS orchestration. This approach eliminates the need to build Muto from source and provides a fast path to get started with containerized deployment.

**What You'll Accomplish:**
- Deploy Eclipse Muto using pre-built container images
- Configure distributed ROS orchestration with cloud management  
- Deploy and manage ROS stacks remotely via Symphony
- Monitor and troubleshoot the integrated containerized system

**Architecture Overview:**
```
Eclipse Symphony (Cloud Orchestration)
        ↓ MQTT/API
Eclipse Muto Container (Containerized Agent & Gateway)
        ↓ ROS Topics  
Eclipse Muto Composer (Stack Orchestration)
        ↓ Launch System
ROS 2 Nodes & Applications
```

**Container Benefits:**
- **Rapid Deployment**: Pre-built images eliminate compilation time
- **Consistent Environment**: Reproducible deployments across platforms
- **Multi-Architecture**: Support for both AMD64 and ARM64 platforms
- **Self-Contained**: All dependencies included in the container

> **📖 For detailed technical information** about Eclipse Muto's architecture, components, and capabilities, see the [Eclipse Muto Overview](../README.md) document.

## Prerequisites

- **Container Runtime**: Podman or Docker with multi-architecture support
- **Eclipse Symphony Setup**: [Eclipse Symphony Quick Start](https://github.com/eclipse-symphony/symphony/blob/main/docs/README.md) or use the docker-compose setup from [Eclipse Symphony Docker Compose](https://github.com/Eclipse-SDV-Hackathon-Chapter-Three/challenge-mission-update-possible/blob/main/symphony/README.md)
- **Network Access**: Host networking for ROS communication

## Getting Started

This guide walks you through setting up and running Eclipse Muto containers with Eclipse Symphony integration.

### 1. Launch Symphony Infrastructure

First, start the Symphony server and MQTT broker infrastructure using the provided Docker Compose configuration:

```bash
git clone git@github.com:Eclipse-SDV-Hackathon-Chapter-Three/challenge-mission-update-possible.git
# Navigate to the Symphony directory
cd challenge-mission-update-possible/symphony

# Start Symphony API, Portal, and Mosquitto MQTT broker
docker compose up -d
```

This launches:
- **Symphony API**: `http://localhost:8082` - REST API for orchestration management
- **Symphony Portal**: `http://localhost:3000` - Web interface for monitoring
- **Mosquitto MQTT**: `localhost:1883` - Message broker for communication

**Verify Symphony is running:**
```bash
# Test Symphony API connectivity
curl http://localhost:8082/v1alpha2/greetings
# Expected response: "Hello from Symphony K8s control plane (S8C)"
```

### 2. Prepare Muto Configuration

#### Clone Muto Repository for Configuration Files
```bash
# Clone the repository to get configuration and launch files
git clone https://github.com/eclipse-muto/muto.git
cd muto
```

#### Main Configuration (`config/muto.yaml`)

The main configuration file defines the ROS parameters and connections for Muto components. This YAML file configures:

**Purpose**: Central configuration for all Muto components including ROS topics, MQTT connections, digital twin integration, and Symphony orchestration settings.

**Key Configuration Areas**:
- **ROS Topics**: Internal communication channels between Muto components (stack, twin, agent-gateway topics)
- **Digital Twin Integration**: Eclipse Ditto connection parameters for device state synchronization
- **Vehicle Identity**: Namespace, name, type, and attributes that uniquely identify the robot/vehicle
- **Symphony Integration**: Cloud orchestration settings including API endpoints, MQTT broker configuration, and provider settings
- **MQTT Connectivity**: Broker connection parameters for both local and cloud communication

> **📝 Configuration Reference**: See [`config/muto.yaml`](../config/muto.yaml) for the complete configuration template with all available parameters and default values.

**Customization**: Edit the configuration file to match your environment, particularly the Symphony host settings, vehicle identity parameters, and MQTT broker addresses.

#### Launch Configuration (`launch/muto.launch.py`)

The launch file orchestrates the startup of all Muto components and defines their relationships.

**Purpose**: ROS 2 launch file that coordinates the startup sequence of Muto components including the Agent, Gateway, Composer, and Core modules with proper parameter configuration.

**Key Features**:
- **Component Orchestration**: Manages startup order and dependencies between Muto components
- **Parameter Management**: Passes configuration from `muto.yaml` to individual ROS nodes
- **Launch Arguments**: Supports runtime configuration through environment variables like `MUTO_LAUNCH_ARGS`
- **Conditional Startup**: Enables/disables components based on configuration (e.g., Symphony integration)
- **Namespace Handling**: Manages ROS namespaces for multi-vehicle deployments

> **📝 Launch Reference**: See [`launch/muto.launch.py`](../launch/muto.launch.py) for the complete launch configuration that coordinates all Muto components.

### 3. Launch Muto Container System

#### Run Complete Muto System in Container

```bash
# Navigate to muto repository root (where config/ and launch/ directories exist)
cd muto

# Launch Muto container with Symphony integration
podman run --rm -it \
  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble
```

#### Launch Arguments Explained

- **`MUTO_LAUNCH`**: Path to the launch file inside the container
- **`MUTO_LAUNCH_ARGS`**: ROS 2 launch arguments:
  - **`vehicle_namespace`**: Hierarchical namespace for device organization (e.g., `org.eclipse.muto.test`)
  - **`vehicle_name`**: Unique device identifier (e.g., `test-robot-debug`)
  - **`enable_symphony`**: Enable/disable Symphony integration (`true`/`false`)
- **Volume Mounts**:
  - **`-v $(pwd)/launch:/work/launch:ro`**: Mount local launch files (read-only)
  - **`-v $(pwd)/config:/work/config:ro`**: Mount local configuration files (read-only)
- **`--network host`**: Use host networking for ROS communication

#### Alternative: Run with Custom Configuration

If you want to use different configuration files:

```bash
# Create custom config directory
mkdir -p ./my-config
cp config/muto.yaml ./my-config/

# Edit ./my-config/muto.yaml as needed
# ... make your changes ...

# Launch with custom config
podman run --rm -it \
  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.my.company vehicle_name:=my-robot enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/my-config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble
```

### 4. Verify System Operation

#### Monitor Container Logs

The container will display logs from all Muto components:

```
[INFO] [launch]: All log files can be found below /root/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [muto_agent-1]: process started with pid [23]
[INFO] [mqtt-2]: process started with pid [25]
[INFO] [commands-3]: process started with pid [27]
[INFO] [twin-4]: process started with pid [29]
[INFO] [muto_composer-5]: process started with pid [31]
[INFO] [compose_plugin-6]: process started with pid [33]
[INFO] [native_plugin-7]: process started with pid [35]
[INFO] [launch_plugin-8]: process started with pid [37]
[INFO] [symphony_provider-9]: process started with pid [39]
```

#### Expected Successful Startup Messages

Look for these key messages indicating successful initialization:

```
[muto_agent-1] [INFO] [timestamp] [muto.agent]: Muto Agent started successfully
[mqtt-2] [INFO] [timestamp] [muto.gateway]: MQTT connection established to sandbox.composiv.ai:1883
[twin-4] [INFO] [timestamp] [muto.core_twin]: Device registered successfully
[symphony_provider-9] [INFO] [timestamp] [muto.muto_symphony_provider]: Symphony Provider started successfully
```

#### Check System Health in Another Terminal

While the container is running, you can verify the system in another terminal:

```bash
# Test Symphony API health
curl http://localhost:8082/v1alpha2/greetings

# Check if MQTT broker is accessible
mosquitto_pub -h localhost -p 1883 -t "test/topic" -m "test message"
```

### 5. Deploy ROS Stacks via Symphony

Symphony uses a three-tier orchestration model: **Target** → **Solution** → **Instance**

This hierarchical approach separates device registration, software package definition, and deployment requests into distinct, manageable components.

#### Target Registration (Automatic)

**Target**: Represents the deployment destination (device/robot) with its capabilities, components, and communication topology.

- **Automatic Registration**: The Target is **automatically registered** when the Muto agent starts up
- **Target Name**: Uses the same name as the vehicle (configured in `config/muto.yaml` as the `name` parameter)
- **Device Identity**: Targets represent physical or logical devices that can execute solutions
- **Capabilities**: Defines what components and services the device can provide

> **📝 Target Reference**: See [`samples/symphony/target.json`](samples/symphony/target.json) for the target definition structure that gets automatically registered.

#### Solution Definition (via Symphony API)

**Solution**: Defines a versioned software package containing ROS stack definitions as base64-encoded payloads.

- **Stack Payload**: Solutions contain the ROS stack definition as a **base64-encoded payload**
- **API Creation**: Solutions are defined using the Symphony API rather than configuration files
- **Versioning**: Multiple solution versions can coexist for different deployment scenarios
- **Component Packaging**: Encapsulates the "what" - the software stack definition that can be deployed

**Creating Solutions**:
```bash
# Example: Create a solution using the provided script
cd docs/samples/config
./define-solution.sh talker-listener.json
```

> **📝 Solution Script**: See [`docs/samples/symphony/define-solution.sh`](../docs/samples/symphony/define-solution.sh) for an example of how solutions can be created via the Symphony API with base64-encoded stack data.

#### Instance Deployment (via Symphony API)  

**Instance**: Defines a deployment request that links a specific Solution version to a specific Target.

- **Deployment Binding**: Creates the logical connection between software (Solution) and hardware (Target)  
- **API Creation**: Instances are also defined using the Symphony API
- **Runtime Deployment**: Represents the "where and when" - actual deployments of solutions to targets

**Creating Instances**:
```bash
# Example: Create an instance using the provided script  
cd docs/samples/config
./define-instance.sh
```

> **📝 Instance Script**: See [`docs/samples/symphony/define-instance.sh`](../docs/samples/symphony/define-instance.sh) for an example of how instances can be created via the Symphony API.

**Target-Solution-Instance Benefits**:
- **Reusability**: Solutions can be deployed to multiple targets
- **Versioning**: Multiple solution versions can coexist  
- **Flexibility**: Different solution versions can be deployed to different targets
- **Traceability**: Clear deployment history and relationships
- **Separation of Concerns**: Device capabilities, software packages, and deployments are managed independently

#### Complete Deployment Workflow

The typical deployment workflow using the provided scripts:

1. **Start Muto Container**: Target gets automatically registered with Symphony
2. **Create Solution**: Use the solution script to package your ROS stack
3. **Deploy Instance**: Use the instance script to deploy the solution to the target

```bash
# Step 1: Start Muto container (Target auto-registers)
podman run --rm -it \
  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble

# Step 2: Create solution (in another terminal)
cd src/agent/config
./define-solution.sh talker-listener.json

# Step 3: Deploy instance  
./define-instance.sh
```

> **📁 Reference Files**: 
> - **Stack Definition**: [`docs/samples/symphony/talker-listener.json`](../docs/samples/symphony/talker-listener.json) - ROS node collection to be deployed
> - **Solution Script**: [`docs/samples/symphony/define-solution.sh`](../docs/samples/symphony/define-solution.sh) - Creates Symphony solution via API
> - **Instance Script**: [`docs/samples/symphony/instance.json`](../docs/samples/symphony/instance.json) - Instance definition
> - **Instance Script**: [`docs/samples/symphony/define-instance.sh`](../docs/samples/symphony/define-instance.sh) - Deploys instance via API

### 6. System Monitoring & Troubleshooting

#### Container Management

```bash
# View running containers
podman ps

# Stop the Muto container (Ctrl+C in the terminal running it)
# Or from another terminal:
podman stop $(podman ps -q --filter ancestor=ghcr.io/eclipse-muto/muto:ros2-humble)

# Check container logs if run in background
podman logs <container-id>
```

#### Health Checks

```bash
# Symphony API health
curl http://localhost:8082/v1alpha2/greetings

# MQTT broker connectivity  
mosquitto_pub -h localhost -p 1883 -t "test/topic" -m "test message"

# Check Symphony targets
curl -H "Content-Type: application/json" \
     http://localhost:8082/v1alpha2/targets
```

#### Common Container Issues

- **Port conflicts**: Ensure ports 1883, 3000, 8082 are not in use by other services
- **Volume mount errors**: Verify that `$(pwd)/config` and `$(pwd)/launch` directories exist
- **Network connectivity**: Use `--network host` for ROS communication
- **Container startup failures**: Check that the container image is available locally

#### Debugging Container Issues

```bash
# Run container with shell access for debugging
podman run --rm -it \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble \
  /bin/bash

# Inside container, you can inspect the environment:
# ls -la /work/
# cat /work/config/muto.yaml  # View mounted configuration
# ros2 node list
```

### 7. Advanced Container Configuration

#### Using Different Container Tags

```bash
# Use specific architecture
podman run --rm -it --arch arm64 \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble

# Use version-specific tag
podman run --rm -it \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble-ubuntu22.04-cpu-2c9bac4-fixed3
```

#### Custom Launch Files

```bash
# Create custom launch file
mkdir -p ./my-launch
cp launch/muto.launch.py ./my-launch/custom-muto.launch.py

# Edit custom launch file as needed
# ... make your modifications ...

# Use custom launch file
podman run --rm -it \
  -e MUTO_LAUNCH=/work/launch/custom-muto.launch.py \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
  -v $(pwd)/my-launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble
```

#### Running in Background

```bash
# Run container in background (detached mode)
podman run -d \
  --name muto-system \
  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble

# Monitor logs
podman logs -f muto-system

# Stop background container
podman stop muto-system
podman rm muto-system
```


---

> **📚 Next Steps**: For comprehensive technical details about Eclipse Muto's architecture, advanced features, and development guide, see the [Eclipse Muto Overview](../README.md) document.