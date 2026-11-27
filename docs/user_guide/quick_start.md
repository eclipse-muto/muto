# Eclipse Muto - Quick Start with Eclipse Muto Containers and Eclipse Symphony

This guide shows you how to deploy Eclipse Muto using pre-built container images, with Eclipse Symphony integration for distributed ROS orchestration. This approach eliminates the need to build Muto from source and provides a fast path to get started with containerized deployment.

## Why Choose Container Deployment?

Container deployment offers several key advantages:

- **⚡ Fast Setup**: Pre-built images eliminate compilation time
- **🔄 Consistent Environment**: Reproducible deployments across different systems
- **🏗️ Multi-Architecture**: Native support for AMD64 and ARM64 architectures  
- **📦 Self-Contained**: All dependencies included, minimal host requirements
- **🚀 Production Ready**: Optimized images for production deployments
- **🎯 Easy Scaling**: Simple horizontal scaling for fleet deployments


**What You'll Accomplish:**
- Deploy Eclipse Muto using pre-built container images
- Configure distributed ROS orchestration with cloud management  
- Find where to go next to deploy ROS stacks via Symphony (see Next Steps)
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

> **📖 For detailed technical information** about Eclipse Muto's architecture, components, and capabilities, see the [Eclipse Muto Overview](../project_overview.md) document.

## Prerequisites

- **Container Runtime**: Podman (the commands below use `podman`; Docker also works, replace `podman` with `docker`)
  - **Ubuntu 24.04 (Jazzy)**: `podman-compose` for the Symphony infrastructure (`sudo apt install podman podman-compose`)
  - **Ubuntu 22.04 (Humble)**: `podman-compose` does not work with the Podman 3.4 shipped with Ubuntu 22.04, so the Symphony infrastructure is started with Docker (`docker compose`)
- **Eclipse Symphony Setup**: [Eclipse Symphony Quick Start](https://github.com/eclipse-symphony/symphony/blob/main/docs/README.md) or use the docker-compose setup from [Eclipse Symphony Docker Compose](../samples/symphony/docker-compose.yaml)
- **Network Access**: Host networking for ROS communication
- **ROS 2 distribution**: Pre-built images are published for Humble (`ros2-humble`) and Jazzy (`ros2-jazzy`); you select one with the `MUTO_IMAGE` variable in step 3.
- **Command-line tools**: `git`, `curl`, and `mosquitto_pub` (`sudo apt install mosquitto-clients`) for the verification steps

## Getting Started

This guide walks you through setting up and running Eclipse Muto containers with Eclipse Symphony integration.

### 1. Launch Symphony Infrastructure

First, start the Symphony server and MQTT broker infrastructure using the provided Docker Compose configuration:

```bash
# Clone the repository (including submodules) and navigate to the Symphony sample
git clone --recurse-submodules https://github.com/eclipse-muto/muto.git
cd muto/docs/samples/symphony/

# Start Symphony API, Portal, and Mosquitto MQTT broker
# Ubuntu 24.04 (Jazzy):
podman-compose up -d
# Ubuntu 22.04 (Humble): podman-compose does not work with Podman 3.4, use Docker instead
# docker compose up -d
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

#### Use the Muto Repository for Configuration Files

The configuration and launch files come from the repository you cloned in step 1. Go back to the repository root:

> **Note:** If you are still inside `muto/docs/samples/symphony` from step 1, use `cd ../../..` instead of `cd muto/`.

```bash
# Navigate to the Muto directory
cd muto/
# (if you are still in muto/docs/samples/symphony from step 1: cd ../../..)
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

> **📝 Configuration Reference**: See [`config/muto.yaml`](../../config/muto.yaml) for the complete configuration template with all available parameters and default values.

**Customization**: The launch file passes a set of connection settings from environment variables *after* `muto.yaml`, so for these settings the environment variable always wins and editing them in `muto.yaml` has no effect. Set them with `-e` on the `podman run` command instead (see step 3):

| Setting | Environment variable | Default |
|---|---|---|
| MQTT broker (gateway) | `MUTO_HOST`, `MUTO_PORT` | `sandbox.composiv.ai`, `1883` |
| Digital Twin (Ditto) | `MUTO_TWIN_URL`, `MUTO_TWIN_USER`, `MUTO_TWIN_PASSWORD` | `https://ditto:ditto@sandbox.composiv.ai`, `ditto`, `ditto` |
| Symphony API | `MUTO_SYMPHONY_API_URL`, `MUTO_SYMPHONY_USER`, `MUTO_SYMPHONY_PASSWORD` | `http://localhost:8082/v1alpha2/`, `admin`, empty |
| Symphony MQTT | `MUTO_SYMPHONY_HOST`, `MUTO_SYMPHONY_PORT`, `MUTO_SYMPHONY_BROKER_ADDRESS` | `localhost`, `1883`, `tcp://mosquitto:1883` |
| Symphony misc. | `MUTO_SYMPHONY_PROPERTIES_GROUP`, `SYMPHONY_CLIENT_ID` | `fleet-1`, `symphony` |

The vehicle identity is set with the `vehicle_namespace` and `vehicle_name` launch arguments (step 3). All other settings (topic names, vehicle `type` and `attributes`, MQTT `keep_alive`/`user`/`password`, other `symphony_*` values) are taken from `muto.yaml`.

#### Launch Configuration (`launch/muto.launch.py`)

The launch file orchestrates the startup of all Muto components and defines their relationships.

**Purpose**: ROS 2 launch file that coordinates the startup sequence of Muto components including the Agent, Gateway, Composer, and Core modules with proper parameter configuration.

**Key Features**:
- **Component Orchestration**: Manages startup order and dependencies between Muto components
- **Parameter Management**: Passes configuration from `muto.yaml` to individual ROS nodes
- **Environment Variables**: Supports runtime configuration of connection settings through environment variables like `MUTO_HOST` and `MUTO_TWIN_URL` (see Customization above); in the container, `MUTO_LAUNCH_ARGS` passes launch arguments to this file
- **Conditional Startup**: Enables/disables components based on launch arguments (e.g., `enable_symphony` for the Symphony integration)
- **Namespace Handling**: Manages ROS namespaces for multi-vehicle deployments

> **📝 Launch Reference**: See [`launch/muto.launch.py`](../../launch/muto.launch.py) for the complete launch configuration that coordinates all Muto components.

### 3. Launch Muto Container System

#### Run Complete Muto System in Container

```bash
# Make sure you are in the muto repository root (where config/ and launch/ directories exist)
ls config/muto.yaml launch/muto.launch.py

# Select the image for your ROS 2 distribution
MUTO_IMAGE=ghcr.io/eclipse-muto/muto:ros2-humble   # or: ghcr.io/eclipse-muto/muto:ros2-jazzy

# Launch Muto container with Symphony integration
podman run --rm -it --name muto \
  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  $MUTO_IMAGE
```

#### Launch Arguments Explained

- **`MUTO_IMAGE`**: Container image to run, `ros2-humble` or `ros2-jazzy`
- **`--name muto`**: Container name, used to stop the container in step 5
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

If you want to use different configuration files and connection settings:

```bash
# Create custom config directory
mkdir -p ./my-config
cp config/muto.yaml ./my-config/

# Edit ./my-config/muto.yaml as needed (topics, vehicle type/attributes, ...)
# ... make your changes ...

# Launch with custom config; connection settings are passed as environment variables
# (see the Customization table in step 2), e.g. your own MQTT broker and Ditto instance
podman run --rm -it --name muto \
  -e MUTO_HOST=my-broker.example.com \
  -e MUTO_TWIN_URL=http://my-ditto.example.com:8080 \
  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.my.company vehicle_name:=my-robot enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/my-config:/work/config:ro \
  --network host \
  $MUTO_IMAGE
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
[INFO] [provision_plugin-7]: process started with pid [35]
[INFO] [launch_plugin-8]: process started with pid [37]
[INFO] [symphony_provider-9]: process started with pid [39]
```

#### Expected Successful Startup Messages

Look for these key messages indicating successful initialization:

```
[muto_agent-1] [INFO] [timestamp] [muto.agent]: Muto Agent started successfully
[mqtt-2] [INFO] [timestamp] [muto.gateway]: MQTT connection established to sandbox.composiv.ai:1883
[symphony_provider-9] [INFO] [timestamp] [muto.muto_symphony_provider]: Symphony Provider started successfully
[symphony_provider-9] [INFO] [timestamp] [muto.muto_symphony_provider]: Successfully registered target 'test-robot-debug'
```

> **Note:** With the default `config/muto.yaml` the Digital Twin (Eclipse Ditto) points at an external sandbox. If it is not reachable you will see a repeating `[twin-4] [WARN] [muto.core_twin]: Twin Server ping failed: ...` warning. This does not affect the Symphony flow in this guide and can be ignored; pass `-e MUTO_TWIN_URL=<your Ditto URL>` to `podman run` to use your own Ditto instance and remove it.

While the container is running, you can verify the system from another terminal with the [Health Checks](#health-checks) in step 5.

### 5. System Monitoring & Troubleshooting

#### Container Management

```bash
# View running containers
podman ps

# Stop the Muto container (Ctrl+C in the terminal running it)
# Or from another terminal:
podman stop muto

# Stop the Symphony infrastructure (from docs/samples/symphony/)
podman-compose down        # Ubuntu 24.04 (Jazzy)
# docker compose down      # Ubuntu 22.04 (Humble)
```

#### Health Checks

```bash
# Symphony API health
curl http://localhost:8082/v1alpha2/greetings

# MQTT broker connectivity  
mosquitto_pub -h localhost -p 1883 -t "test/topic" -m "test message"

# Check that Muto registered itself as a Symphony target (the API requires a token)
# The commands below use jq; install it if it is not installed yet
command -v jq >/dev/null || sudo apt install -y jq

TOKEN=$(curl -s -X POST -H "Content-Type: application/json" \
     -d '{"username":"admin","password":""}' \
     http://localhost:8082/v1alpha2/users/auth | jq -r .accessToken)

curl -s -H "Authorization: Bearer $TOKEN" \
     http://localhost:8082/v1alpha2/targets/registry | jq '.[].metadata.name'
# Expected output includes: "test-robot-debug"
# (the registry lists every target registered so far, e.g. also "my-robot" if you ran the custom configuration example)
```



---

 **🚀 Deploy a Stack via Symphony**: With Muto running, you can deploy ROS stacks to it through Symphony. See the talker-listener example in [`docs/samples/talker-listener/README.md`](../samples/talker-listener/README.md) and the [Running Examples](running_examples.md) guide.

 **📚 Next Steps**: For comprehensive technical details about Eclipse Muto's architecture, advanced features, and development guide, see the [Eclipse Muto Overview](../project_overview.md) document.

