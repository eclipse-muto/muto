# Eclipse Muto - Composer Model Reference

The **Model** subsystem provides the core data structures and abstractions for representing, manipulating, and deploying ROS 2 software stacks within Eclipse Muto's orchestration framework.

## Overview

The Composer Model system provides comprehensive abstractions for:
- **Stack Definitions**: Complete software stack representation with lifecycle management
- **Node Modeling**: Individual ROS 2 node configuration and parameter management
- **Composable Containers**: Efficient composable node container orchestration
- **Parameter Systems**: Dynamic parameter resolution with expression evaluation
- **State Management**: Stack comparison, merging, and incremental deployment capabilities

## Architecture

```
model/
├── stack.py        # Core stack representation and operations
├── node.py         # ROS 2 node modeling and lifecycle
├── param.py        # Parameter management and resolution
└── composable.py   # Composable container management
```

## Core Components

### Stack Model (`stack.py`)

**Purpose**: Central abstraction representing complete software stacks with comprehensive lifecycle management, state comparison, and deployment capabilities.

The Stack class serves as the primary orchestration unit within Eclipse Muto, encapsulating all aspects of a deployable software configuration.

#### Stack Class Architecture

```python
class Stack:
    def __init__(self, manifest={}, parent=None):
        self.manifest = manifest
        self.parent = parent
        self.name = manifest.get('name', '')
        self.context = manifest.get('context', '')
        self.stackId = manifest.get('stackId', '')
        
        # Core components
        self.param = []      # Stack-level parameters
        self.arg = {}        # Launch arguments
        self.node = []       # Regular ROS 2 nodes
        self.composable = [] # Composable containers
        self.stack = []      # Nested stacks
```

**Core Attributes**:
- **`name`**: Human-readable stack identifier
- **`context`**: Deployment context (e.g., "production", "development")
- **`stackId`**: Unique stack identifier (e.g., "org.eclipse.muto:example:v1.0")
- **`manifest`**: Original stack definition dictionary
- **`parent`**: Reference to parent stack for nested configurations

#### Stack Initialization and Lifecycle

**`initialize()`**: Process manifest and create internal representations
```python
def initialize(self):
    """Initialize stack elements (nodes, composable nodes, parameters)."""
    
    # Initialize parameters
    params = []
    for pDef in self.manifest.get('param', []):
        params.append(param.Param(self, pDef))
    self.param = params
    
    # Initialize nodes
    self.node = []
    for nDef in self.manifest.get('node', []):
        sn = node.Node(self, nDef)
        self.node.append(sn)
    
    # Initialize composable containers
    self.composable = []
    for cDef in self.manifest.get('composable', []):
        sn = composable.Container(self, cDef)
        self.composable.append(sn)
```

#### Stack Operations

**State Comparison and Analysis**:

```python
def compare_nodes(self, other):
    """Compare nodes between current and target stack."""
    current_nodes = {f"{n.namespace}/{n.name}": n for n in self.flatten_nodes([])}
    other_nodes = {f"{n.namespace}/{n.name}": n for n in other.flatten_nodes([])}
    
    common_keys = current_nodes.keys() & other_nodes.keys()
    added_keys = other_nodes.keys() - current_nodes.keys()
    removed_keys = current_nodes.keys() - other_nodes.keys()
    
    return (
        [current_nodes[key] for key in common_keys],
        [other_nodes[key] for key in added_keys],
        [current_nodes[key] for key in removed_keys]
    )

def compare_composable(self, other):
    """Compare composable containers between stacks."""
    current_composables = {f"{c.namespace}/{c.name}": c for c in self.flatten_composable([])}
    other_composables = {f"{c.namespace}/{c.name}": c for c in other.flatten_composable([])}
    
    # Return common, added, removed composables
    # Implementation details in source code
```

**Stack Merging**:

```python
def merge(self, other):
    """Merge current stack with another stack for incremental deployment."""
    merged = Stack(manifest={}, parent=None)
    
    # Merge attributes
    self._merge_attributes(merged, other)
    
    # Merge components
    self._merge_nodes(merged, other)
    self._merge_composables(merged, other) 
    self._merge_params(merged, other)
    
    merged.manifest = merged.toManifest()
    return merged
```

#### Deployment Operations

**Stack Launch**:
```python
def launch(self, launcher):
    """Launch the complete stack using ROS 2 launch system."""
    launch_description = LaunchDescription()
    
    try:
        # Handle composable containers first
        self.handle_composable_nodes(self.composable, launch_description, launcher)
        
        # Handle regular nodes
        self.handle_regular_nodes(self.node, launch_description, launcher)
        
    except Exception as e:
        self.logger.error(f'Stack launching failed: {e}')
        raise
    
    launcher.start(launch_description)
    
    # Handle managed node lifecycle
    all_nodes = self.node + [cn for c in self.composable for cn in c.nodes]
    self.handle_managed_nodes(all_nodes, verb='start')

def apply(self, launcher):
    """Apply incremental changes to running stack."""
    self.kill_diff(launcher, self)  # Remove obsolete nodes
    self.launch(launcher)           # Launch new/modified nodes
```

**Node Handling**:
```python
def handle_regular_nodes(self, nodes, launch_description, launcher):
    """Process regular ROS 2 nodes for launch."""
    for n in nodes:
        if n.action == STARTACTION or (n.action == NOACTION and self.should_node_run(n, launcher)):
            launch_description.add_action(Node(
                package=n.pkg,
                executable=n.exec,
                name=n.name,
                namespace=n.namespace,
                output=n.output,
                parameters=n.ros_params,
                arguments=n.args.split(),
                remappings=self.process_remaps(n.remap)
            ))

def handle_composable_nodes(self, composable_containers, launch_description, launcher):
    """Process composable containers and their contained nodes."""
    for c in composable_containers:
        # Create composable node descriptions
        node_desc = [
            ComposableNode(
                package=cn.pkg, 
                plugin=cn.plugin, 
                name=cn.name, 
                namespace=cn.namespace,
                parameters=cn.ros_params,
                remappings=self.process_remaps(cn.remap)
            )
            for cn in c.nodes 
            if cn.action == STARTACTION or (cn.action == NOACTION and self.should_node_run(cn, launcher))
        ]
        
        if node_desc:
            container = ComposableNodeContainer(
                name=c.name,
                namespace=c.namespace,
                package=c.package,
                executable=c.executable,
                output=c.output,
                composable_node_descriptions=node_desc,
            )
            launch_description.add_action(container)
```

#### Expression Resolution

**Dynamic Value Resolution**:
```python
def resolve_expression(self, value=""):
    """Resolve expressions like $(find package), $(arg name), $(env VAR)."""
    if isinstance(value, str):
        # $(find package) resolution
        value = re.sub(r'\$\(find ([^)]+)\)', 
                      lambda m: get_package_share_directory(m.group(1)), value)
        
        # $(arg name) resolution  
        value = re.sub(r'\$\(arg ([^)]+)\)', 
                      lambda m: self.arg.get(m.group(1), {}).get('value', ''), value)
        
        # $(env VAR) resolution
        value = re.sub(r'\$\(env ([^)]+)\)', 
                      lambda m: os.getenv(m.group(1), ''), value)
    
    return value
```

#### Stack Serialization

**Manifest Generation**:
```python
def toManifest(self):
    """Convert stack to manifest dictionary for persistence or transmission."""
    manifest = self.toShallowManifest()
    
    # Add component manifests
    for p in self.param:
        manifest["param"].append(p.toManifest())
    
    for a in self.arg:
        manifest["arg"].append(self.arg[a])
    
    for s in self.stack:
        manifest["stack"].append(s.toShallowManifest())
    
    for n in self.node:
        manifest["node"].append(n.toManifest())
    
    for c in self.composable:
        manifest["composable"].append(c.toManifest())
    
    return manifest
```

### Node Model (`node.py`)

**Purpose**: Comprehensive ROS 2 node representation with lifecycle management, parameter handling, and advanced configuration support.

#### Node Class Structure

```python
class Node:
    def __init__(self, stack, manifest={}, container=None):
        self.stack = stack
        self.container = container  # Parent container for composable nodes
        self.manifest = manifest
        
        # Core node properties
        self.pkg = manifest.get('pkg', '')
        self.exec = manifest.get('exec', '')
        self.plugin = manifest.get('plugin', '')  # For composable nodes
        self.name = manifest.get('name', '')
        self.namespace = manifest.get('namespace', os.getenv('MUTONS', ''))
        
        # Configuration
        self.param = [param.Param(stack, pDef) for pDef in manifest.get('param', [])]
        self.remap = manifest.get('remap', [])
        self.args = stack.resolve_expression(manifest.get('args', ''))
        self.output = manifest.get('output', 'both')
        
        # Lifecycle and actions
        self.lifecycle = manifest.get('lifecycle', '')
        self.action = manifest.get('action', '')  # 'start', 'stop', 'load', etc.
        
        # Advanced features
        self.launch_prefix = manifest.get('launch-prefix', None)
        self.env = manifest.get('env', [])
        self.ros_args = manifest.get('ros_args', '')
```

**Key Attributes**:
- **Core Identity**: pkg, exec, name, namespace
- **Composable Support**: plugin field for composable node identification
- **Parameter Management**: Dynamic parameter resolution and ROS param conversion
- **Lifecycle Support**: Lifecycle node management capabilities
- **Action Control**: Node-level action specification (start, stop, load)

#### Node Configuration Processing

**Parameter Processing**:
```python
# Convert Muto parameters to ROS 2 format
self.ros_params = [
    {key: value} 
    for p in self.param 
    if isinstance(p.value, dict) 
    for key, value in p.value.items()
]

# Process remappings
self.remap_args = [
    (stack.resolve_expression(rm['from']), stack.resolve_expression(rm['to'])) 
    for rm in self.remap
]
```

**Lifecycle Management**:
```python
def configure(self):
    """Configure lifecycle node."""
    if self.lifecycle:
        # Implement lifecycle node configuration
        pass

def activate(self):
    """Activate lifecycle node."""
    if self.lifecycle:
        # Implement lifecycle node activation
        pass
```

#### Node Manifest Serialization

```python
def toManifest(self):
    """Convert node to manifest dictionary."""
    manifest = {
        "name": self.name,
        "pkg": self.pkg,
        "exec": self.exec,
        "namespace": self.namespace,
        "output": self.output,
        "args": self.args,
    }
    
    # Add optional fields
    if self.plugin:
        manifest["plugin"] = self.plugin
    if self.lifecycle:
        manifest["lifecycle"] = self.lifecycle
    if self.action:
        manifest["action"] = self.action
    if self.param:
        manifest["param"] = [p.toManifest() for p in self.param]
    if self.remap:
        manifest["remap"] = self.remap
    
    return manifest
```

### Parameter Model (`param.py`)

**Purpose**: Sophisticated parameter management with dynamic resolution, file loading, and command execution capabilities.

#### Parameter Class

```python
class Param:
    def __init__(self, stack, manifest=None):
        if manifest is None:
            manifest = {}
        
        self.stack = stack
        self.manifest = manifest
        self.name = manifest.get('name', '')
        self.value = self._resolve_value(manifest, stack)
        self.sep = manifest.get('sep', '')
        self.from_file = manifest.get('from', '')
        self.namespace = manifest.get('namespace', '/')
        self.command = manifest.get('command', '')
```

#### Parameter Resolution Methods

**Dynamic Value Resolution**:
```python
def _resolve_value(self, manifest, stack):
    """Resolve parameter value from various sources."""
    
    # File-based parameters
    if 'from' in manifest:
        return self._resolve_from_file(stack.resolve_expression(manifest['from']))
    
    # Command-based parameters
    if 'command' in manifest:
        return self._execute_command(stack.resolve_expression(manifest['command']))
    
    # Direct value
    return self._parse_value(manifest.get('value'))

def _resolve_from_file(self, file_path):
    """Load parameter from YAML file."""
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
            return data
    except Exception as e:
        print(f"Error loading parameter file {file_path}: {e}")
        return {}

def _execute_command(self, command):
    """Execute command and return output as parameter value."""
    try:
        result = subprocess.run(
            shlex.split(command), 
            capture_output=True, 
            text=True, 
            check=True
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"Command execution failed: {e}")
        return ""
```

**Type Conversion**:
```python
def _parse_value(self, value):
    """Parse and convert parameter values to appropriate types."""
    if value is None:
        return None
    
    # String values
    if isinstance(value, str):
        # Boolean conversion
        if value.lower() in ('true', 'false'):
            return value.lower() == 'true'
        
        # Numeric conversion
        try:
            if '.' in value:
                return float(value)
            return int(value)
        except ValueError:
            return value  # Keep as string
    
    return value  # Return as-is for complex types
```

#### Parameter Examples

**Basic Parameters**:
```json
{
  "param": [
    {"name": "use_sim_time", "value": "True"},
    {"name": "robot_name", "value": "muto_robot_01"},
    {"name": "update_rate", "value": 10.0}
  ]
}
```

**File-based Parameters**:
```json
{
  "param": [
    {"from": "$(find navigation)/config/costmap.yaml"},
    {"from": "/opt/muto/config/robot_params.yaml"}
  ]
}
```

**Command-based Parameters**:
```json
{
  "param": [
    {"command": "hostname", "name": "robot_hostname"},
    {"command": "cat /proc/cpuinfo | grep cores", "name": "cpu_cores"}
  ]
}
```

### Composable Container Model (`composable.py`)

**Purpose**: Efficient management of ROS 2 composable node containers with lifecycle control and node composition capabilities.

#### Container Class

```python
class Container:
    def __init__(self, stack, manifest=None):
        if manifest is None:
            manifest = {}
        
        self.stack = stack
        self.manifest = manifest
        
        # Container properties
        self.package = manifest.get('package', '')
        self.executable = manifest.get('executable', '')
        self.name = manifest.get('name', '')
        self.namespace = manifest.get('namespace', os.getenv('MUTONS', default=''))
        self.output = manifest.get('output', 'screen')
        
        # Contained nodes
        self.nodes = [
            node.Node(stack, nDef, self) 
            for nDef in manifest.get('node', [])
        ]
        
        # Configuration
        self.remap = manifest.get('remap', [])
        self.action = manifest.get('action', '')
```

**Key Features**:
- **Container Management**: Full container lifecycle management
- **Node Composition**: Dynamic composition of multiple nodes within container
- **Resource Efficiency**: Shared process space for multiple nodes
- **Namespace Isolation**: Proper namespace management for contained nodes

#### Container Operations

**Manifest Serialization**:
```python
def toManifest(self):
    """Convert container to manifest dictionary."""
    return {
        "package": self.package,
        "executable": self.executable,
        "name": self.name,
        "namespace": self.namespace,
        "node": [n.toManifest() for n in self.nodes],
        "output": self.output,
        "remap": self.remap,
        "action": self.action
    }
```

#### Composable Node Integration

Composable containers work seamlessly with the Stack model for efficient deployment:

```python
# In Stack.handle_composable_nodes()
def handle_composable_nodes(self, composable_containers, launch_description, launcher):
    for c in composable_containers:
        # Create composable node descriptions
        node_desc = [
            ComposableNode(
                package=cn.pkg,
                plugin=cn.plugin,  # Plugin class for composable node
                name=cn.name,
                namespace=cn.namespace,
                parameters=cn.ros_params,
                remappings=self.process_remaps(cn.remap)
            )
            for cn in c.nodes
            if self.should_node_run(cn, launcher)
        ]
        
        if node_desc:
            container = ComposableNodeContainer(
                name=c.name,
                namespace=c.namespace,
                package=c.package,
                executable=c.executable,
                output=c.output,
                composable_node_descriptions=node_desc,
            )
            launch_description.add_action(container)
```

## Stack Definition Examples

### Basic Stack with Regular Nodes

```json
{
  "name": "Basic Navigation Stack",
  "context": "production",
  "stackId": "org.eclipse.muto:navigation:v1.0",
  "param": [
    {"name": "use_sim_time", "value": "False"},
    {"name": "robot_base_frame", "value": "base_link"}
  ],
  "arg": [
    {"name": "map_file", "value": "/maps/warehouse.yaml"}
  ],
  "node": [
    {
      "name": "map_server",
      "pkg": "nav2_map_server", 
      "exec": "map_server",
      "param": [
        {"name": "yaml_filename", "value": "$(arg map_file)"},
        {"name": "topic", "value": "map"},
        {"name": "frame_id", "value": "map"}
      ]
    },
    {
      "name": "amcl",
      "pkg": "nav2_amcl",
      "exec": "amcl",
      "param": [
        {"from": "$(find navigation_config)/config/amcl.yaml"}
      ]
    }
  ]
}
```

### Composable Node Stack

```json
{
  "name": "Efficient Sensor Processing",
  "stackId": "org.eclipse.muto:sensors:composable:v1.0",
  "composable": [
    {
      "name": "sensor_container",
      "namespace": "/sensors",
      "package": "rclcpp_components",
      "executable": "component_container",
      "node": [
        {
          "pkg": "camera_drivers",
          "plugin": "camera_drivers::CameraNode",
          "name": "front_camera",
          "param": [
            {"name": "device_id", "value": 0},
            {"name": "frame_id", "value": "camera_front"}
          ]
        },
        {
          "pkg": "lidar_drivers", 
          "plugin": "lidar_drivers::LidarNode",
          "name": "main_lidar",
          "param": [
            {"name": "port", "value": "/dev/ttyUSB0"},
            {"name": "frame_id", "value": "lidar"}
          ]
        }
      ]
    }
  ]
}
```

### Complex Multi-Stack Configuration

```json
{
  "name": "F1tenth Multiagent Gym",
  "context": "simulation",
  "stackId": "org.eclipse.muto.sandbox:f1tenth-multiagent-gym:v1.0",
  "stack": [
    {"thingId": "org.eclipse.muto.sandbox:racecar1.launch"},
    {"thingId": "org.eclipse.muto.sandbox:racecar2.launch"},
    {"thingId": "org.eclipse.muto.sandbox:racecar3.launch"}
  ],
  "arg": [
    {"name": "map", "value": "$(find f1tenth_gym_ros)/maps/Spielberg_map.yaml"}
  ],
  "node": [
    {
      "name": "map_server",
      "pkg": "nav2_map_server",
      "exec": "map_server",
      "param": [
        {"name": "yaml_filename", "value": "$(arg map)"},
        {"name": "use_sim_time", "value": "True"}
      ]
    },
    {
      "name": "gym_bridge", 
      "pkg": "f1tenth_gym_ros",
      "exec": "gym_bridge",
      "param": [
        {"from": "$(find f1tenth_gym_ros)/config/sim.yaml"}
      ]
    }
  ]
}
```

### Advanced Parameter Resolution

```json
{
  "name": "Dynamic Configuration Stack",
  "param": [
    {"name": "robot_id", "command": "hostname"},
    {"name": "cpu_count", "command": "nproc"},
    {"name": "nav_config", "from": "$(find robot_config)/config/navigation_$(env ROBOT_TYPE).yaml"},
    {"name": "use_hardware", "value": "$(env HARDWARE_ENABLED)"}
  ],
  "node": [
    {
      "name": "controller",
      "pkg": "robot_controller",
      "exec": "controller_node",
      "param": [
        {"name": "robot_id", "value": "$(param robot_id)"},
        {"name": "thread_count", "value": "$(param cpu_count)"}
      ]
    }
  ]
}
```

## Model Integration Patterns

### Stack Factory Pattern

```python
class StackFactory:
    @staticmethod
    def create_from_manifest(manifest_dict):
        """Create stack from manifest dictionary."""
        return Stack(manifest=manifest_dict)
    
    @staticmethod
    def create_from_json(json_string):
        """Create stack from JSON string."""
        import json
        manifest = json.loads(json_string)
        return Stack(manifest=manifest)
    
    @staticmethod
    def create_from_file(file_path):
        """Create stack from file."""
        import yaml
        with open(file_path, 'r') as f:
            manifest = yaml.safe_load(f)
        return Stack(manifest=manifest)
```

### Stack Composition Pattern

```python
def compose_stacks(base_stack, overlay_stack):
    """Compose multiple stacks into single deployable unit."""
    
    # Create composite stack
    composite = Stack()
    composite.name = f"{base_stack.name} + {overlay_stack.name}"
    composite.stackId = f"composite:{base_stack.stackId}+{overlay_stack.stackId}"
    
    # Merge components
    composite.node = base_stack.node + overlay_stack.node
    composite.composable = base_stack.composable + overlay_stack.composable
    composite.param = base_stack.param + overlay_stack.param
    
    # Resolve conflicts
    composite = resolve_conflicts(composite)
    
    return composite
```

### Dynamic Stack Modification

```python
def modify_stack_runtime(stack, modifications):
    """Apply runtime modifications to stack configuration."""
    
    for mod in modifications:
        if mod['type'] == 'add_node':
            new_node = node.Node(stack, mod['node_config'])
            stack.node.append(new_node)
        
        elif mod['type'] == 'update_param':
            for param in stack.param:
                if param.name == mod['param_name']:
                    param.value = mod['new_value']
        
        elif mod['type'] == 'remove_node':
            stack.node = [n for n in stack.node if n.name != mod['node_name']]
    
    # Regenerate manifest
    stack.manifest = stack.toManifest()
    return stack
```

## Best Practices

### Stack Design Guidelines

1. **Atomic Operations**: Design stacks for atomic deployment and rollback
2. **Parameter Isolation**: Use namespacing to prevent parameter conflicts
3. **Resource Management**: Consider resource requirements in stack design
4. **Lifecycle Awareness**: Plan for proper node lifecycle management

### Model Usage Patterns

```python
# Good: Use factory methods for consistency
stack = StackFactory.create_from_manifest(manifest)

# Good: Validate before operations  
if stack.validate():
    stack.launch(launcher)

# Good: Handle errors gracefully
try:
    merged_stack = base_stack.merge(overlay_stack)
except MergeConflictError as e:
    logger.error(f"Stack merge failed: {e}")
    handle_merge_conflict(e)
```

This comprehensive model documentation provides complete coverage of Eclipse Muto's stack representation system, from basic data structures through advanced composition and deployment patterns.