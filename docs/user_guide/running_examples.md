# Eclipse Muto - Running Examples

This guide provides step-by-step instructions for running practical Eclipse Muto examples. Each example demonstrates key features and use cases with complete deployment instructions.

## Prerequisites

Before running these examples, ensure you have:

- ✅ Eclipse Muto installed and running (see [Quick Start Guide](./quick_start.md))
- ✅ Eclipse Symphony services running (if using cloud orchestration)
- ✅ ROS 2 Humble or later installed and sourced
- ✅ Basic familiarity with ROS concepts and JSON

## Example 1: Simple Talker-Listener Stack

**What you'll learn**: Basic stack deployment, node configuration, and message flow

**Time to complete**: 5 minutes

**Complexity**: Beginner

### Overview
This example deploys the classic ROS 2 talker-listener demo using Muto's declarative stack management. You'll see how Muto transforms traditional ROS launch procedures into remote-manageable stack definitions.

### Step 1. Deploy Talker-Listener Stack via Symphony

Symphony uses a three-tier orchestration model: **Target** → **Solution** → **Instance**

This hierarchical approach separates device registration, software package definition, and deployment requests into distinct, manageable components.

#### Target Registration (Automatic)

**Target**: Represents the deployment destination (device/robot) with its capabilities, components, and communication topology.

- **Automatic Registration**: The Target is **automatically registered** when the Muto agent starts up
- **Target Name**: Uses the same name as the vehicle (configured in `config/muto.yaml` as the `name` parameter)
- **Device Identity**: Targets represent physical or logical devices that can execute solutions
- **Capabilities**: Defines what components and services the device can provide

> **📝 Target Reference**: See [`samples/symphony/target.json`](../samples/symphony/target.json) for the target definition structure that gets automatically registered.

#### Solution Definition (via Symphony API)

**Solution**: Defines a versioned software package containing ROS stack definitions as base64-encoded payloads.

- **Stack Payload**: Solutions contain the ROS stack definition as a **base64-encoded payload**
- **API Creation**: Solutions are defined using the Symphony API rather than configuration files
- **Versioning**: Multiple solution versions can coexist for different deployment scenarios
- **Component Packaging**: Encapsulates the "what" - the software stack definition that can be deployed

**Creating Solutions**:

**Create Stack Definition**
```bash
# Create a simple talker-listener stack
cat > talker-listener-json.json  << 'EOF'
{
  "metadata": {
    "name": "Muto Simple Talker-Listener Stack",
    "description": "A simple talker-listener stack example using demo_nodes_cpp package.",
    "content_type": "stack/json"
  },
  "launch": {
      "node": [
        {
          "name": "talker",
          "pkg": "demo_nodes_cpp",
          "exec": "talker"
        },
        {
          "name": "listener",
          "pkg": "demo_nodes_cpp",
          "exec": "listener"
        }
      ]
  }
}
EOF
```

**Define Solution via Symphony API**:
```bash
# Example: Create a solution using the provided script
./docs/samples/symphony/define-solution.sh  ./docs/samples/talker-listener/talker-listener-json.json 
```

> **📝 Solution Script**: See [`define-solution.sh`](../samples/symphony/define-solution.sh) for an example of how solutions can be created via the Symphony API with base64-encoded stack data.

#### Instance Deployment (via Symphony API)  

**Instance**: Defines a deployment request that links a specific Solution version to a specific Target.

- **Deployment Binding**: Creates the logical connection between software (Solution) and hardware (Target)  
- **API Creation**: Instances are also defined using the Symphony API
- **Runtime Deployment**: Represents the "where and when" - actual deployments of solutions to targets

**Creating Instances**:

**Create Instance Data**:
```bash
# Create a simple talker-listener stack
cat > talker-listener-json-instance.json  << 'EOF'
{
    "metadata": {
        "name": "talker-listener-json-instance",
        "labels": {
            "muto": "demo"
        }
    },
    "spec": {
        "solution": "talker-listener-json:1",
        "target": {
            "name": "test-robot-debug"
        }
    }
}
EOF
```
**Create Instance Definition via Symphony API **:

```bash
# Example: Create an instance using the provided script  
./docs/samples/symphony/define-instance.sh ./docs/samples/talker-listener/talker-listener-json-instance.json 
```

> **📝 Define Instance Script**: See [`define-instance.sh`](../samples/symphony/define-instance.sh) for an example of how instances can be created via the Symphony API.




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
./docs/samples/symphony/define-solution.sh  ./docs/samples/talker-listener/talker-listener-json.json 

# Step 3: Deploy instance  
./docs/samples/symphony/define-instance.sh ./docs/samples/talker-listener/talker-listener-json-instance.json 

```

> **📁 Reference Files**: 
> - **Stack Definition**: [`docs/samples/symphony/`](../samples/talker-listener/talker-listener-json.json) - ROS node collection to be deployed
> - **Solution Script**: [`docs/samples/symphony/define-solution.sh`](../samples/symphony/define-solution.sh) - Creates Symphony solution via API
> - **Instance Script**: [`docs/samples/symphony/define-instance.sh`](../samples/symphony/define-instance.sh) - Deploys instance via API


### Step 2: Verify Deployment
```bash
# Check that nodes are running
ros2 node list | grep -E "(talker|listener)"
# Expected output:
# /demo/talker
# /demo/listener

# Listen to the messages
ros2 topic echo /chatter
# Expected output:
# data: "Hello World: 1"
# ---
# data: "Hello World: 2"
# ---
```


### Step 5: Stop the Stack
#### Instance Removal (via Symphony API)  

**Instance**: Removes a deployment that links a specific Solution version to a specific Target.

- **Deployment Binding**: Creates the logical connection between software (Solution) and hardware (Target)  

**Stopping/Removing Solution from Target**:
You do this by deleting the instance
```bash
# Example: Delete an instance using the provided script  
./docs/samples/symphony/delete-instance.sh ./docs/samples/talker-listener/talker-listener-json-instance.json 
```

> **📝 Delete Instance Script**: See [`delete-instance.sh`](../samples/symphony/delete-instance.sh) for an example of how instances can be deleted via the Symphony API.

**Target-Solution-Instance Benefits**:
- **Reusability**: Solutions can be deployed to multiple targets
- **Versioning**: Multiple solution versions can coexist  
- **Flexibility**: Different solution versions can be deployed to different targets
- **Traceability**: Clear deployment history and relationships
- **Separation of Concerns**: Device capabilities, software packages, and deployments are managed independently

### Expected Output
When successful, you should see:
- Talker node publishing "Hello World: X" messages
- Listener node receiving and processing messages
- Nodes appearing in ROS node list with `/demo/` namespace
- Stack status showing "running" state

---

## Example 2: Custom Talker-Listener with a workspace launch

**What you'll learn**: 

**Time to complete**: 15 minutes

**Complexity**: Intermediate

### Overview
This example demonstrates deploying a custom implementatio of the talker-listener stack from code using an archive.

### Step 1: Develop code
We provide you the complete implementation here
[Sample Custom Talker Listener Code](../samples/talker-listener/sample-stack/)


### Step 2: Create Stack Manifest with a Archive of the workspace
```bash
# Create a  talker-listener stack with embedded archive (ws)
cat > talker-listener-xarchive.json  << 'EOF'
{
    "metadata": {
        "name": "Muto Simple Talker-Listener Stack",
        "description": "A simple talker-listener stack example using demo_nodes_cpp package.",
        "content_type": "stack/archive"
    },
    "launch": {
        "data": "H4sIAAAAAAAAA+1de3PbxhH[ REMOVED FOR CALRITY]F46SUAKAAAA==",
        "properties": {
            "algorithm": "sha256",
            "checksum": "553fd2dc7d0eb41e7d65c467d358e7962d3efbb0e2f2e4f8158e926a081f96d0",
            "launch_file": "launch/talker_listener.launch.py",
            "command": "launch",
            "launch_args": [
                {
                    "name": "arg1",
                    "default": "val1"
                }
            ],
            "ros_args": [],
            "flatten": true
        }
    }

}
EOF
```

### Step 3: Define a solution

**Define Solution via Symphony API**:
```bash
# Example: Create a solution using the provided script
./docs/samples/symphony/define-solution.sh  ./docs/samples/talker-listener/talker-listener-xarchive.json 
```

### Step 4: Define an instance 
**Create Instance Data**:
```bash
# Create a simple talker-listener stack
cat > talker-listener-xarchive-instance.json  << 'EOF'
{
    "metadata": {
        "name": "talker-listener-xarchive-instance",
        "labels": {
            "muto": "demo"
        }
    },
    "spec": {
        "solution": "talker-listener-xarchive:1",
        "target": {
            "name": "test-robot-debug"
        }
    }
}
EOF
```
**Create Instance Definition via Symphony API **:

```bash
# Example: Create an instance using the provided script  
./docs/samples/symphony/define-instance.sh ./docs/samples/talker-listener/talker-listener-xarchive-instance.json 
```


## Getting Help

If you encounter issues:

1. **Check Prerequisites**: Ensure all required packages are installed
2. **Verify Configuration**: Confirm Muto configuration matches your environment
3. **Review Logs**: Check ROS logs and Muto component outputs
4. **Test Incrementally**: Start with simple examples and add complexity
5. **Community Support**: Ask questions in [GitHub Discussions](https://github.com/eclipse-muto/muto/discussions)

## Next Steps

📚 **Technical Deep Dive**: Read [Reference Documentation](../reference/readme.md)

🛠️ **Developer Guide**: Follow [Developer Guide](../developer_guide/readme.md) to add features

---

**Congratulations!** You've successfully run Eclipse Muto examples and seen how declarative stack management simplifies ROS deployment. These examples provide a foundation for deploying your own robotic applications with Muto's powerful orchestration capabilities.