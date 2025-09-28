# Eclipse Muto - Running from Source

This guide explains how to run Eclipse Muto components directly from source code, including configuration, debugging, and development workflows.

## Prerequisites

Before running from source, ensure you have:
- ✅ Completed [Development Environment Setup](./development_environment_setup.md)
- ✅ Successfully built Eclipse Muto from [Building from Source](./building_from_source.md)
- ✅ Source workspace at `~/muto_ws` with built packages

## Quick Start

### Verify Build and Environment
```bash
# Navigate to workspace
cd ~/muto_ws

# Source the workspace
source install/setup.bash

# Verify packages are available
ros2 pkg list | grep -E "agent|composer|core|muto_msgs"

# Verify executables are available
# Expected: agent, mcomposer, core, muto_msgs

```

## Step 1: Configuration and Launch Setup

### 1.1 Development Configuration
Review the muto [configuration yaml (muto.yaml)](../../config/muto.yaml) and [launch (muto.launch.py)](../../launch/muto.launch.py) and change for your own purposes
```bash
# Create development configuration directory
mkdir -p config

# Create development configuration file
cp  <muto repo>/config/muto.yaml config
```

### Edit config file
Make a copy of [muto configuration yaml (muto.yaml)](../../config/muto.yaml) file  and change for your own purposes. For example to enable symphony providert and change its URL. Edit the block

```yaml
/**:
  ros__parameters:

    [REMOVED FOR CLARITY]

    symphony_enabled: True
    symphony_name: "hack2025-01"
    symphony_target_name: "muto-device-001"
    symphony_topic_prefix: "muto"
    symphony_host: "localhost"
    symphony_port: 1883
    symphony_keep_alive: 60
    symphony_prefix: muto
    symphony_user: "admin"
    symphony_password: ""

    # Symphony API configuration
    symphony_api_url: "http://localhost:8082/v1alpha2/"
    symphony_provider_name: "providers.target.mqtt"
    symphony_broker_address: "tcp://mosquitto:1883"
    symphony_client_id: "symphony"
    symphony_request_topic: "coa-request"
    symphony_response_topic: "coa-response"
    symphony_timeout_seconds: "30"
    symphony_auto_register: False

    [REMOVED FOR CLARITY]

```
### Edit launch file
Make a copy of [muto launch (muto.launch.py)](../../launch/muto.launch.py) and change for your own purposes. 
```bash
# Create development configuration directory
mkdir -p launch

# Create development configuration file
cp  <muto repo>/launch/muto.launch.py launch
```

## Step 2: Run muto

### 2.1 Running Muto Core
```bash
# Terminal 1: Start Muto Core
cd ~/muto_ws
source install/setup.bash

# Run with default configuration
ros2 launch launch/muto.launch.py \
    vehicle_namespace:=org.eclipse.muto.test \
    vehicle_name:=test-robot-debug \
    enable_symphony:=true \
    log_level:=DEBUG

# Run with custom configuration
ros2 launch launch/muto.launch.py \
    vehicle_namespace:=org.eclipse.muto.test \
    vehicle_name:=test-robot-debug \
    enable_symphony:=true \
    log_level:=DEBUG \
    config_file:=$MUTO_CONFIG_PATH


```

Eclipse Muto is now ready to run from source with comprehensive development, debugging, and monitoring capabilities!