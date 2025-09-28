# Eclipse Muto - Composer Reference

**Composer** is the heart of Eclipse Muto edge orchestration, managing asynchronous state processing, pipelines, and plugins for comprehensive software deployment and lifecycle management.

## Overview

The Composer component acts as the central orchestration engine for Eclipse Muto, responsible for:
- Processing stack deployment requests from the Agent
- Managing the complete software deployment lifecycle
- Orchestrating plugin execution through configurable pipelines
- Handling workspace provisioning, building, and launching
- Providing introspection and monitoring capabilities

## Architecture

The Composer architecture consists of several interconnected components:

```
Composer
├── Main Orchestrator (muto_composer.py)
├── Plugins/
│   ├── Compose Plugin      # Stack composition and validation
│   ├── Provision Plugin    # Workspace setup and building
│   └── Launch Plugin       # Stack execution management
├── Workflow/
│   ├── Pipeline           # Execution orchestration
│   ├── Router             # Message routing
│   ├── Launcher           # ROS 2 launch integration
│   └── Safe Evaluator     # Secure expression evaluation
├── Model/
│   ├── Stack             # Stack representation
│   ├── Node              # ROS 2 node modeling
│   ├── Param             # Parameter management
│   └── Composable        # Composable node handling
├── Introspection/
│   ├── Introspector      # System analysis
│   └── Traverser         # Graph traversal utilities
└── Utils/
    └── Stack Parser      # Stack format parsing
```

## Core Functionality

### 1. Stack Processing Pipeline

The Composer processes stacks through a configurable pipeline system:

```yaml
# Example pipeline configuration
pipelines:
  - name: "deployment_pipeline"
    condition: "start"
    steps:
      - plugin: "compose_plugin"
        condition: "always"
      - plugin: "provision_plugin" 
        condition: "has_archive or has_repository"
      - plugin: "launch_plugin"
        condition: "always"
```

### 2. Stack Format Support

Composer supports multiple stack formats:

** Plain JSON Stack Format:**
```json
{
  "name": "Example Stack",
  "stackId": "example:stack:v1.0",
  "node": [
    {
      "name": "example_node",
      "pkg": "example_package",
      "exec": "example_executable"
    }
  ]
}
```

**Stack Manifest - JSON Format:**
```json
{
  "metadata": {
        "name": "Example Stack",
        "stackId": "example:stack:v1.0",
        "content_type": "stack/json"
  },
  "launch": {
    "node": [
        {
        "name": "example_node",
        "pkg": "example_package",
        "exec": "example_executable"
        }
    ]
  }
}
```

**Stack Manifest - Archive Format:**
```json
{
  "metadata": {
    "name": "Archived Stack",
    "content_type": "stack/archive"
  },
  "launch": {
    "data": "base64_encoded_archive_data",
    "properties": {
      "filename": "workspace.tar.gz",
      "flatten": true,
      "launch_file": "launch/example.launch.py"
    }
  }
}
```

**Stack Manifest - Git Repository Format (planned):**
```json
{
  "metadata": {
    "name": "Repository Stack",
    "content_type": "stack/git"

  },
  "launch": {
    "url": "https://github.com/example/stack.git",
    "branch": "main",
    "path": "myworkspace",
    "launch_file": "launch/example.launch.py"
  }
}
```

### 3. Plugin Architecture

Composer uses an extensible plugin architecture where each plugin handles specific aspects of stack deployment:

#### Plugin Interface
All plugins implement a common service interface pattern:
```python
class MutoPlugin(Node):
    def handle_request(self, request, response):
        # Process request
        # Update response
        return response
```

## Component Reference

### Main Orchestrator (`muto_composer.py`)

**Purpose**: Central coordination node managing the overall stack deployment process.

**Key Responsibilities**:
- Receives stack deployment requests via ROS 2 topics
- Determines execution path based on stack characteristics  
- Orchestrates plugin execution through pipelines
- Manages stack state transitions and persistence
- Provides bootstrap functionality for device initialization

**Topics**:
- **Subscriptions**: `stack` (MutoAction) - Stack deployment requests
- **Publications**: 
  - `composed_stack` (StackManifest) - Processed stack manifests
  - `raw_stack` (String) - Raw stack JSON data

**Services**:
- **Client**: `core_twin` (CoreTwin) - Digital twin communication

### Plugins

#### Compose Plugin (`compose_plugin.py`)

**Purpose**: Handles stack composition, validation, and initial processing.

**Key Responsibilities**:
- Validates incoming stack definitions
- Applies composition rules and transformations
- Publishes composed stacks to downstream components
- Manages stack metadata and context

**Service Interface**: `ComposePlugin`
- **Request**: `start` (bool), `input` (PlanManifest)
- **Response**: `output` (PlanManifest), `success` (bool), `err_msg` (string)

**Key Features**:
- Stack format validation
- Metadata extraction and processing
- Context-aware stack transformation
- Error handling and reporting

#### Provision Plugin (`provision_plugin.py`)

**Purpose**: Comprehensive workspace setup, dependency management, and build orchestration.

**Key Responsibilities**:
- Workspace provisioning from multiple sources (Git, archives, URLs)
- Dependency resolution and installation via rosdep
- Automated building using colcon
- State tracking and caching for efficiency
- Cleanup and maintenance operations

**Service Interface**: `ProvisionPlugin`
- **Request**: `start` (bool), `input` (PlanManifest)  
- **Response**: `output` (PlanManifest), `success` (bool), `err_msg` (string)

**Supported Sources**:
- **Git Repositories** (planned): Clone, branch checkout, submodule handling
- **Manifest - Archive Files**: Base64 encoded TAR/ZIP extraction with validation
- **Manifest - JSON**: Json in a manifest
- **Simple - JSON**: JSON


#### Launch Plugin (`launch_plugin.py`)

**Purpose**: Manages stack execution, ROS 2 launch integration, and process lifecycle.

**Key Responsibilities**:
- ROS 2 launch file execution and management
- Process monitoring and health checking
- Graceful shutdown and cleanup
- Launch parameter injection and configuration
- Integration with workspace environments

**Service Interface**: `LaunchPlugin`
- **Request**: `start` (bool), `input` (PlanManifest)
- **Response**: `output` (PlanManifest), `success` (bool), `err_msg` (string)

**Launch Process**:
1. **Environment Setup**: Source workspace environment
2. **Parameter Resolution**: Process launch parameters
3. **Launch Execution**: Start ROS 2 launch process
4. **Monitoring**: Track process health and status
5. **Cleanup**: Graceful shutdown on termination

**Key Features**:
- Dynamic launch file discovery
- Parameter injection and customization
- Process group management
- Signal handling for graceful shutdown
- Launch process isolation

### Workflow Components

#### Pipeline (`pipeline.py`)

**Purpose**: Orchestrates sequential execution of plugins based on configuration and conditions.

**Key Features**:
- **Conditional Execution**: Execute steps based on runtime conditions
- **Context Management**: Pass execution context between steps
- **Error Handling**: Robust error recovery and reporting
- **Parallel Execution**: Support for concurrent plugin execution
- **Configuration-Driven**: YAML-based pipeline definitions

**Pipeline Configuration**:
```yaml
pipelines:
  - name: "standard_deployment"
    condition: "method == 'start'"
    steps:
      - plugin: "compose_plugin"
        condition: "always"
        timeout: 30
      - plugin: "provision_plugin"
        condition: "has_repository or has_archive"
        timeout: 300
      - plugin: "launch_plugin" 
        condition: "success(provision_plugin) or skip_provision"
        timeout: 60
```

#### Router (`router.py`)

**Purpose**: Intelligent message routing and pipeline selection based on message content and context.

**Key Responsibilities**:
- Message analysis and classification
- Pipeline selection based on stack characteristics
- Context variable extraction and management
- Error routing and handling

**Routing Logic**:
- Analyzes incoming stack messages
- Determines appropriate pipeline based on content type
- Sets execution context variables
- Routes to selected pipeline for processing


#### Safe Evaluator (`safe_evaluator.py`)

**Purpose**: Secure evaluation of expressions and scripts within stack definitions.

**Security Features**:
- Restricted execution environment
- Whitelist-based function access
- Input sanitization and validation
- Resource limitation and timeout enforcement
- Audit logging of all evaluations

**Supported Operations**:
- Mathematical expressions
- String manipulation
- Conditional logic
- Variable substitution
- Environment variable access (controlled)

### Model Components

#### Stack (`stack.py`)

**Purpose**: Complete stack definition modeling with validation and transformation capabilities.

**Key Features**:
- **Schema Validation**: JSON Schema-based validation
- **Format Conversion**: Convert between different stack formats
- **Metadata Management**: Handle stack metadata and versioning
- **Dependency Analysis**: Analyze stack dependencies and requirements

**Stack Attributes**:
- `name`: Human-readable stack name
- `stackId`: Unique stack identifier
- `nodes`: List of ROS 2 nodes to launch
- `parameters`: Global and node-specific parameters
- `launch`: Launch configuration and artifacts
- `metadata`: Stack metadata and annotations

#### Node (`node.py`)

**Purpose**: ROS 2 node representation with comprehensive configuration support.

**Node Attributes**:
- `name`: Node instance name
- `package`: ROS 2 package name
- `executable`: Executable name within package
- `namespace`: Node namespace
- `parameters`: Node-specific parameters
- `remappings`: Topic and service remappings
- `arguments`: Command-line arguments

**Advanced Features**:
- Parameter validation and type checking
- Namespace resolution and inheritance
- Remapping conflict detection
- Launch argument processing

#### Param (`param.py`)

**Purpose**: Parameter management with type safety and validation.

**Parameter Types**:
- Basic types: string, int, float, bool
- Arrays: string_array, int_array, float_array, bool_array
- Files: parameter files and YAML configurations
- Dynamic: runtime parameter resolution

**Features**:
- Type validation and conversion
- Default value handling
- Parameter file loading
- Namespace-aware parameter resolution

#### Composable (`composable.py`)

**Purpose**: ROS 2 composable node management and lifecycle.

**Key Features**:
- Container management and discovery
- Component loading and unloading
- Lifecycle state management
- Inter-component communication setup

### Introspection Tools

#### Introspector (`introspector.py`)

**Purpose**: System analysis and monitoring capabilities for debugging and optimization.

**Analysis Capabilities**:
- **Launch Tree Analysis**: Visualize launch file hierarchies
- **Dependency Mapping**: Map node and package dependencies
- **Resource Usage**: Monitor CPU, memory, and network usage
- **Performance Metrics**: Collect execution timing and statistics

**Monitoring Features**:
- Real-time system state monitoring
- Historical performance tracking
- Alert generation for anomalies
- Diagnostic report generation

#### Traverser (`traverser.py`)

**Purpose**: Graph traversal utilities for complex system analysis.

**Traversal Algorithms**:
- Depth-first search (DFS)
- Breadth-first search (BFS)
- Topological sorting
- Cycle detection
- Shortest path analysis

**Applications**:
- Dependency resolution ordering
- Launch sequence optimization
- Circular dependency detection
- Impact analysis for changes

### Utilities

#### Stack Parser (`stack_parser.py`)

**Purpose**: Comprehensive parsing and format conversion for various stack formats.

**Supported Formats**:
- **JSON Stack**: Native Muto stack format
- **ROS 2 Launch**: Launch file-based stacks
- **Legacy Formats**: Backward compatibility support
- **URL-based**: Remote stack definitions

**Key Features**:
- Format auto-detection
- Schema validation
- Error reporting with context
- Lossy and lossless conversion options

## Usage Examples

### Basic Stack Deployment

```python
# Publish stack deployment request
stack_action = MutoAction()
stack_action.method = "start"
stack_action.payload = json.dumps({
    "name": "Example Stack",
    "stackId": "example:v1.0",
    "node": [{
        "name": "talker",
        "pkg": "demo_nodes_cpp", 
        "exec": "talker"
    }]
})

publisher.publish(stack_action)
```

### Archive-based Deployment

```python
# Deploy from archive
archive_stack = {
    "metadata": {
        "name": "Archive Stack",
        "content_type": "stack/archive"
    },
    "launch": {
        "data": base64_encoded_tar,
        "properties": {
            "filename": "workspace.tar.gz",
            "subdir": "ros_workspace",
            "flatten": True
        }
    }
}

# Publish deployment
stack_msg = MutoAction()
stack_msg.method = "start"
stack_msg.payload = json.dumps(archive_stack)
```


## Extension Points

### Custom Plugins

Create custom plugins by implementing the plugin interface:

```python
class CustomPlugin(Node):
    def __init__(self):
        super().__init__("custom_plugin")
        self.service = self.create_service(
            CustomPluginSrv, 
            "muto_custom", 
            self.handle_request
        )
    
    def handle_request(self, request, response):
        # Implementation here
        return response
```

### Pipeline Extensions

Add custom pipeline steps:

```yaml
pipelines:
  - name: "custom_pipeline"
    steps:
      - plugin: "custom_plugin"
        condition: "custom_condition"
        parameters:
          custom_param: "value"
```

This comprehensive reference provides detailed information about Eclipse Muto's Composer component, covering architecture, functionality, configuration, and usage patterns for effective edge orchestration.