# Eclipse Muto - Messages Reference

**Messages** defines all Eclipse Muto ROS 2 message types and service interfaces, providing the core communication protocols and data structures that enable seamless interaction between all Muto components.

## Overview

The Messages package serves as the communication foundation for Eclipse Muto, providing:
- Standardized message types for inter-component communication
- Service interface definitions for plugin architecture
- Type-safe data structures for stack definitions and device management
- Protocol abstractions for gateway and digital twin integration
- Extensible message formats supporting multiple deployment scenarios

## Architecture

The Messages package is organized into three main categories:

```
Messages
├── msg/                    # ROS 2 Message Types
│   ├── CommandInput.msg       # Command execution input
│   ├── CommandOutput.msg      # Command execution output
│   ├── Gateway.msg           # Gateway communication protocol
│   ├── MutoAction.msg        # Central action message
│   ├── MutoActionMeta.msg    # Action metadata
│   ├── PlanManifest.msg      # Plan execution manifest
│   ├── PluginResponse.msg    # Plugin response format
│   ├── StackManifest.msg     # Stack definition manifest
│   ├── Thing.msg            # Digital twin thing representation
│   └── ThingHeaders.msg     # Thing metadata headers
├── srv/                    # ROS 2 Service Definitions
│   ├── CommandPlugin.srv     # Command plugin interface
│   ├── ComposePlugin.srv     # Composition plugin interface
│   ├── CoreTwin.srv         # Digital twin service interface
│   ├── LaunchPlugin.srv     # Launch plugin interface
│   ├── NativePlugin.srv     # Native plugin interface
│   └── ProvisionPlugin.srv  # Provision plugin interface
└── build/                  # Build Configuration
    ├── CMakeLists.txt       # CMake build configuration
    └── package.xml          # ROS 2 package definition
```

## Message Types Reference

### Core Action Messages

#### MutoAction.msg

**Purpose**: Central message type for all Muto action communications, supporting multiple stack formats and execution contexts.

**Message Definition**:
```
string context
string method  
string payload
MutoActionMeta meta
```

**Field Descriptions**:
- **`context`**: Execution context or namespace identifier
- **`method`**: Action method (`start`, `stop`, `apply`, `status`)
- **`payload`**: JSON-serialized action data (stack definition, command, etc.)
- **`meta`**: Metadata for response routing and correlation

**Usage Patterns**:

*Stack Deployment Action*:
```python
action = MutoAction()
action.context = "org.eclipse.muto.device01"
action.method = "start"
action.payload = json.dumps({
    "stackId": "example:stack:v1.0",
    "name": "Example Stack",
    "node": [
        {"name": "talker", "pkg": "demo_nodes_cpp", "exec": "talker"}
    ]
})
action.meta.response_topic = "/muto/stack_response"
```

*Command Execution Action*:
```python
action = MutoAction()
action.context = "command_execution"
action.method = "execute"
action.payload = json.dumps({
    "command": "ros/topic",
    "parameters": {"list": True}
})
```

**Payload Format Variations**:

1. **JSON Stack Format**:
```json
{
  "stackId": "example:stack:v1.0",
  "name": "Example Stack", 
  "node": [
    {
      "name": "example_node",
      "pkg": "example_package",
      "exec": "example_executable",
      "param": [
        {"name": "param_name", "value": "param_value"}
      ]
    }
  ]
}
```

2. **Archive Stack Format**:
```json
{
  "metadata": {
    "name": "Archive Stack",
    "content_type": "stack/archive"
  },
  "launch": {
    "data": "base64_encoded_archive",
    "properties": {
      "filename": "workspace.tar.gz"
    }
  }
}
```

3. **Repository Stack Format**:
```json
{
  "name": "Repo Stack",
  "url": "https://github.com/example/stack.git",
  "branch": "main",
  "launch_description": "launch/stack.launch.py"
}
```

#### MutoActionMeta.msg

**Purpose**: Metadata container for action routing, correlation, and response handling.

**Message Definition**:
```
string id
string response_topic
string correlation_id
builtin_interfaces/Time timestamp
string source
string[] tags
```

**Field Descriptions**:
- **`id`**: Unique action identifier
- **`response_topic`**: Topic for publishing response/result
- **`correlation_id`**: Request-response correlation identifier  
- **`timestamp`**: Action creation timestamp
- **`source`**: Component or service that created the action
- **`tags`**: Additional metadata tags for filtering/routing

**Usage Example**:
```python
meta = MutoActionMeta()
meta.id = str(uuid.uuid4())
meta.response_topic = "/muto/agent_response"
meta.correlation_id = "req_12345"
meta.timestamp = self.get_clock().now().to_msg()
meta.source = "muto_agent"
meta.tags = ["deployment", "production"]
```

### Stack and Plan Messages

#### StackManifest.msg

**Purpose**: Comprehensive stack definition with support for multiple stack formats and execution modes.

**Message Definition**:
```
string name
string context
string stack_id

# Stack 0.0.1 (JSON format)
string type
string stack
PluginResponse result

# Stack 0.0.2 (Repository/Launch format) 
string url
string branch
string launch_description_source
string args
string source
string on_start
string on_kill
```

**Field Descriptions**:
- **`name`**: Human-readable stack name
- **`context`**: Execution context or namespace
- **`stack_id`**: Unique stack identifier
- **`type`**: Stack type ("json", "repository", "archive")
- **`stack`**: JSON-serialized stack definition
- **`result`**: Plugin execution result
- **`url`**: Repository URL for git-based stacks
- **`branch`**: Git branch to checkout
- **`launch_description_source`**: Launch file path within stack
- **`args`**: Launch arguments
- **`source`**: Additional source information
- **`on_start`**: Startup script or command
- **`on_kill`**: Cleanup script or command

**Stack Format Examples**:

*JSON Stack*:
```python
manifest = StackManifest()
manifest.name = "Talker Listener Stack"
manifest.stack_id = "examples:talker-listener:v1.0"
manifest.type = "json"
manifest.stack = json.dumps({
    "node": [
        {"name": "talker", "pkg": "demo_nodes_cpp", "exec": "talker"},
        {"name": "listener", "pkg": "demo_nodes_cpp", "exec": "listener"}
    ]
})
```

*Repository Stack*:
```python
manifest = StackManifest()
manifest.name = "Navigation Stack"
manifest.stack_id = "navigation:basic:v2.0"
manifest.type = "repository"
manifest.url = "https://github.com/nav2-project/nav2_minimal_tb3_sim.git"
manifest.branch = "humble"
manifest.launch_description_source = "launch/tb3_simulation_launch.py"
manifest.args = "use_sim_time:=true"
```

#### PlanManifest.msg

**Purpose**: Execution plan manifest for plugin communication and workflow orchestration.

**Message Definition**:
```
string name
string context
StackManifest current
string url
string branch
```

**Field Descriptions**:
- **`name`**: Plan name or identifier
- **`context`**: Execution context
- **`current`**: Current stack manifest being processed
- **`url`**: Repository URL (if applicable)
- **`branch`**: Branch information (if applicable)

**Usage in Plugins**:
```python
# Plugin request example
request = LaunchPlugin.Request()
request.start = True
request.input.name = "deployment_plan"
request.input.current = stack_manifest
```

### Command Messages

#### CommandInput.msg

**Purpose**: Input message for command execution requests in the plugin system.

**Message Definition**:
```
string command
string payload
```

**Field Descriptions**:
- **`command`**: Command identifier or method name
- **`payload`**: JSON-serialized command parameters

**Command Examples**:

*ROS Topic List*:
```python
cmd_input = CommandInput()
cmd_input.command = "ros/topic"
cmd_input.payload = json.dumps({"action": "list"})
```

*Node Information*:
```python
cmd_input = CommandInput()
cmd_input.command = "ros/node/info"
cmd_input.payload = json.dumps({"node_name": "/talker"})
```

*Parameter Operations*:
```python
cmd_input = CommandInput()
cmd_input.command = "ros/param"
cmd_input.payload = json.dumps({
    "action": "get",
    "node": "/example_node",
    "parameter": "use_sim_time"
})
```

#### CommandOutput.msg

**Purpose**: Output message for command execution results and responses.

**Message Definition**:
```
string stdout
string stderr
int32 return_code
builtin_interfaces/Time timestamp
```

**Field Descriptions**:
- **`stdout`**: Standard output from command execution
- **`stderr`**: Standard error output
- **`return_code`**: Command execution return code (0 = success)
- **`timestamp`**: Command completion timestamp

**Usage Example**:
```python
# Processing command output
if output.return_code == 0:
    print(f"Command succeeded: {output.stdout}")
else:
    print(f"Command failed: {output.stderr}")
```

#### PluginResponse.msg

**Purpose**: Standardized response format for plugin operations with success/error reporting.

**Message Definition**:
```
bool success
string message
string details
builtin_interfaces/Time timestamp
```

**Field Descriptions**:
- **`success`**: Operation success indicator
- **`message`**: Human-readable result message
- **`details`**: Additional details or error information
- **`timestamp`**: Response generation timestamp

**Response Examples**:

*Success Response*:
```python
response = PluginResponse()
response.success = True
response.message = "Stack deployed successfully"
response.details = json.dumps({"nodes_started": 3, "duration": "45s"})
response.timestamp = self.get_clock().now().to_msg()
```

*Error Response*:
```python
response = PluginResponse()
response.success = False
response.message = "Build failed"
response.details = json.dumps({
    "error": "Compilation error in package 'navigation'",
    "build_log": "/tmp/build.log"
})
```

### Gateway and Device Messages

#### Gateway.msg

**Purpose**: Gateway communication protocol for MQTT/cloud integration and message translation.

**Message Definition**:
```
string topic
string payload
ThingHeaders headers
builtin_interfaces/Time timestamp
```

**Field Descriptions**:
- **`topic`**: MQTT topic or routing identifier
- **`payload`**: Message payload data
- **`headers`**: Thing identification and metadata headers
- **`timestamp`**: Message timestamp

**Gateway Usage Patterns**:

*Device Command*:
```python
gateway_msg = Gateway()
gateway_msg.topic = f"{namespace}/{device_name}/command"
gateway_msg.payload = json.dumps({
    "action": "deploy_stack",
    "stack_id": "example:stack:v1.0"
})
gateway_msg.headers.thing_id = f"{namespace}:{device_name}"
```

*Status Update*:
```python
gateway_msg = Gateway()
gateway_msg.topic = f"{namespace}/{device_name}/status"
gateway_msg.payload = json.dumps({
    "state": "online",
    "last_stack": "example:stack:v1.0",
    "uptime": "1234567"
})
```

#### Thing.msg

**Purpose**: Digital twin thing representation with features, properties, and metadata.

**Message Definition**:
```
string thing_id
string[] features
string properties
ThingHeaders headers
```

**Field Descriptions**:
- **`thing_id`**: Unique thing identifier in namespace:name format
- **`features`**: List of available thing features
- **`properties`**: JSON-serialized thing properties
- **`headers`**: Thing headers and metadata

**Thing Representation Example**:
```python
thing = Thing()
thing.thing_id = "org.eclipse.muto:robot-001"
thing.features = ["stack", "status", "telemetry"]
thing.properties = json.dumps({
    "device_type": "mobile_robot",
    "capabilities": ["navigation", "manipulation"],
    "location": "warehouse_a",
    "battery_level": 85
})
```

#### ThingHeaders.msg

**Purpose**: Thing identification headers and metadata for routing and authentication.

**Message Definition**:
```
string thing_id
string namespace
string device_name
string correlation_id
builtin_interfaces/Time timestamp
string[] metadata
```

**Field Descriptions**:
- **`thing_id`**: Complete thing identifier
- **`namespace`**: Thing namespace (e.g., "org.eclipse.muto")
- **`device_name`**: Device name within namespace
- **`correlation_id`**: Request correlation identifier
- **`timestamp`**: Header creation timestamp
- **`metadata`**: Additional metadata key-value pairs

## Service Interface Reference

### Plugin Service Interfaces

#### CommandPlugin.srv

**Purpose**: Service interface for command execution plugins.

**Service Definition**:
```
CommandInput input
MutoActionMeta meta
---
CommandOutput output
PluginResponse response
```

**Usage Pattern**:
```python
# Service client usage
request = CommandPlugin.Request()
request.input.command = "ros/topic"
request.input.payload = json.dumps({"action": "list"})
request.meta.response_topic = "/command_response"

future = self.command_client.call_async(request)
```

#### ComposePlugin.srv

**Purpose**: Service interface for stack composition and validation plugins.

**Service Definition**:
```
bool start
PlanManifest input
---
PlanManifest output
bool success
string err_msg
```

**Usage Pattern**:
```python
# Composition request
request = ComposePlugin.Request()
request.start = True
request.input.current = stack_manifest

response = await self.compose_client.call_async(request)
if response.success:
    composed_stack = response.output
```

#### ProvisionPlugin.srv

**Purpose**: Service interface for workspace provisioning and build management plugins.

**Service Definition**:
```
bool start
PlanManifest input
---
PlanManifest output
bool success  
string err_msg
```

**Provisioning Workflow**:
```python
# Provision workspace
request = ProvisionPlugin.Request()
request.start = True
request.input.current = stack_with_repository

response = await self.provision_client.call_async(request)
if response.success:
    print("Workspace provisioned successfully")
else:
    print(f"Provisioning failed: {response.err_msg}")
```

#### LaunchPlugin.srv

**Purpose**: Service interface for stack launch and execution management plugins.

**Service Definition**:
```
bool start
PlanManifest input
---
PlanManifest output
bool success
string err_msg
```

**Launch Workflow**:
```python
# Launch stack
request = LaunchPlugin.Request()
request.start = True
request.input.current = provisioned_stack

response = await self.launch_client.call_async(request)
if response.success:
    print("Stack launched successfully")
```

#### NativePlugin.srv

**Purpose**: Service interface for native system plugins and extensions.

**Service Definition**:
```
string command
string payload
---
string result
bool success
string error_msg
```

### Core Service Interfaces

#### CoreTwin.srv

**Purpose**: Digital twin service interface for device-to-cloud communication.

**Service Definition**:
```
string input
---
string output
```

**Twin Operations**:

*Get Stack by ID*:
```python
request = CoreTwin.Request()
request.input = "example:stack:v1.0"

response = await self.twin_client.call_async(request)
stack_data = json.loads(response.output)
```

*Get Device Information*:
```python
request = CoreTwin.Request()
request.input = "device_info"

response = await self.twin_client.call_async(request)
device_info = json.loads(response.output)
```

## Message Usage Patterns

### Inter-Component Communication

**Agent to Composer Communication**:
```python
# Agent publishes deployment request
action = MutoAction()
action.method = "start"
action.payload = json.dumps(stack_definition)
agent_pub.publish(action)

# Composer subscribes and processes
def on_stack_callback(self, action_msg):
    payload = json.loads(action_msg.payload)
    # Process stack deployment
```

**Plugin Chain Communication**:
```python
# Plugin A output becomes Plugin B input
compose_response = await self.compose_plugin.call_async(compose_request)

provision_request = ProvisionPlugin.Request()
provision_request.input = compose_response.output
provision_response = await self.provision_plugin.call_async(provision_request)
```

### Error Handling Patterns

**Service Error Handling**:
```python
try:
    response = await service_client.call_async(request)
    if not response.success:
        self.get_logger().error(f"Service failed: {response.err_msg}")
        return None
    return response
except Exception as e:
    self.get_logger().error(f"Service call failed: {e}")
    return None
```

**Message Validation**:
```python
def validate_muto_action(action_msg):
    """Validate MutoAction message format."""
    if not action_msg.method:
        raise ValueError("Missing method in MutoAction")
    
    if not action_msg.payload:
        raise ValueError("Missing payload in MutoAction")
    
    try:
        json.loads(action_msg.payload)
    except json.JSONDecodeError:
        raise ValueError("Invalid JSON in payload")
```


This comprehensive Messages reference provides detailed information about Eclipse Muto's message types and service interfaces, covering all communication protocols, data structures, and usage patterns that enable seamless integration across the Muto ecosystem.