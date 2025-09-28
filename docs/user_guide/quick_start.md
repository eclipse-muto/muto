# Eclipse Muto - Quick Start with Eclipse Muto Containers and Eclipse Symphony

This guide shows you how to deploy Eclipse Muto using pre-built container images for the fastest and most consistent deployment experience. You will set up and deploy Eclipse Muto using pre-built container images with Eclipse Symphony integration for distributed ROS orchestration. This approach eliminates the need to build Muto from source and provides a fast path to get started with containerized deployment.

## Why Choose Container Deployment?

Container deployment offers several key advantages:

- **⚡ Fast Setup**: Pre-built images eliminate compilation time (5-10 minutes vs 30+ minutes)
- **🔄 Consistent Environment**: Reproducible deployments across different systems
- **🏗️ Multi-Architecture**: Native support for AMD64 and ARM64 architectures  
- **📦 Self-Contained**: All dependencies included, minimal host requirements
- **🚀 Production Ready**: Optimized images for production deployments
- **🎯 Easy Scaling**: Simple horizontal scaling for fleet deployments


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
- **Eclipse Symphony Setup**: [Eclipse Symphony Quick Start](https://github.com/eclipse-symphony/symphony/blob/main/docs/README.md) or use the docker-compose setup from [Eclipse Symphony Docker Compose](../../docs/samples/symphony/docker-compose.yaml)
- **Network Access**: Host networking for ROS communication

## Getting Started

This guide walks you through setting up and running Eclipse Muto containers with Eclipse Symphony integration.

### 1. Launch Symphony Infrastructure

First, start the Symphony server and MQTT broker infrastructure using the provided Docker Compose configuration:

```bash
git clone --recurse-submodules  https://github.com/eclipse-muto/muto.git
# Navigate to the Symphony directory
cd docs/samples/symphony/

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
git clone --recurse-submodules  https://github.com/eclipse-muto/muto.git
# Navigate to the Symphony directory
cd muto/

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


### 5. System Monitoring & Troubleshooting

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



---

 **📚 Next Steps**: For comprehensive technical details about Eclipse Muto's architecture, advanced features, and development guide, see the [Eclipse Muto Overview](../project_overview.md) document.

