# Eclipse Muto - Composer Workflow Reference

The **Workflow** subsystem provides the orchestration engine for Eclipse Muto's Composer, managing pipeline execution, message routing, and secure expression evaluation for complex deployment scenarios.

## Overview

The Composer Workflow system coordinates the execution of plugin sequences through configurable pipelines, enabling:
- **Conditional Pipeline Execution**: Execute steps based on runtime conditions and previous step outcomes
- **Service-Based Plugin Architecture**: Integrate plugins through standardized ROS 2 service interfaces
- **Compensation Handling**: Automatic rollback and cleanup on pipeline failures
- **Expression Evaluation**: Secure evaluation of conditional expressions for dynamic workflows
- **Message Routing**: Intelligent routing of actions to appropriate pipelines

## Architecture

```
workflow/
├── pipeline.py           # Core pipeline execution engine
├── router.py            # Action routing and pipeline selection
├── launcher.py          # ROS 2 launch system integration
├── safe_evaluator.py    # Secure expression evaluation
└── schemas/
    └── pipeline_schema.py # Pipeline configuration validation
```

## Core Components

### Pipeline Engine (`pipeline.py`)

**Purpose**: Central orchestration engine that executes sequences of plugin operations with conditional logic and error handling.

**Key Features**:
- **Sequential Execution**: Execute pipeline steps in defined order with dependency management
- **Conditional Logic**: Skip or execute steps based on runtime conditions using safe expression evaluation
- **Service Integration**: Communicate with plugins through standardized ROS 2 service interfaces
- **State Management**: Track step execution state and chain outputs between steps
- **Compensation Handling**: Execute cleanup operations when pipeline steps fail

#### Pipeline Class

```python
class Pipeline:
    def __init__(self, name, steps, compensation=None):
        self.name = name
        self.steps = steps
        self.compensation = compensation or []
        self.plugins = self._load_plugins()
        self.context = {}
        self.logger = rclpy.logging.get_logger(f"pipeline_{name}")
```

**Key Methods**:

**`execute_pipeline(additional_context=None, next_manifest=None)`**
- Execute complete pipeline with optional additional context
- Chain step outputs as inputs for subsequent steps
- Handle conditional execution and error recovery

**`execute_step(step, executor, inputManifest=None)`**
- Execute individual pipeline step using ROS 2 service calls
- Validate step configuration and service availability
- Handle service timeouts and error responses

**`execute_compensation(executor)`**
- Execute compensation steps when pipeline fails
- Cleanup resources and rollback partial deployments

#### Pipeline Configuration

Pipeline behavior is defined through YAML configuration:

```yaml
pipelines:
  - name: "start"
    pipeline:
      - sequence:
        - service: muto_compose
          plugin: ComposePlugin
          name: compose_step
        - service: muto_provision
          plugin: ProvisionPlugin
          name: provision_step
          condition: "should_run_provision == True"
        - service: muto_start_stack
          plugin: LaunchPlugin
          name: start_stack_step
          condition: "compose_step.success == True"
    compensation:
      - service: muto_kill_stack
        plugin: LaunchPlugin
```

**Pipeline Configuration Elements**:

- **`name`**: Pipeline identifier for routing
- **`pipeline`**: Array of sequence objects containing step definitions
- **`sequence`**: Ordered list of steps to execute
- **`compensation`**: Cleanup steps executed on failure

**Step Configuration**:
- **`service`**: ROS 2 service name for plugin communication
- **`plugin`**: Plugin type (ComposePlugin, ProvisionPlugin, LaunchPlugin)
- **`name`**: Step identifier for condition evaluation
- **`condition`**: Optional conditional expression for step execution

#### Pipeline Execution Flow

```
1. Load Pipeline Configuration
   ├── Validate against schema
   ├── Initialize plugin mappings
   └── Create execution context

2. Execute Pipeline Steps
   ├── Evaluate step conditions
   ├── Execute service calls
   ├── Update execution context
   └── Chain step outputs

3. Handle Results
   ├── Success: Complete pipeline
   └── Failure: Execute compensation
```

#### Example Pipeline Execution

```python
# Initialize pipeline
pipeline = Pipeline(
    name="deployment",
    steps=[{
        "sequence": [
            {
                "service": "muto_compose",
                "plugin": "ComposePlugin",
                "name": "compose_step"
            },
            {
                "service": "muto_provision",
                "plugin": "ProvisionPlugin", 
                "name": "provision_step",
                "condition": "compose_step.success == True"
            }
        ]
    }],
    compensation=[{
        "service": "muto_cleanup",
        "plugin": "CleanupPlugin"
    }]
)

# Execute with context
context = {"should_run_provision": True}
pipeline.execute_pipeline(additional_context=context)
```

### Router (`router.py`)

**Purpose**: Intelligent message routing system that directs incoming actions to appropriate pipelines based on action type and content.

**Key Responsibilities**:
- **Action Analysis**: Parse incoming action messages and determine routing requirements
- **Pipeline Selection**: Map actions to configured pipelines based on action type
- **Context Propagation**: Pass execution context to selected pipelines
- **Logging and Monitoring**: Track routing decisions and pipeline selection

#### Router Class

```python
class Router:
    def __init__(self, pipelines):
        self.pipelines = pipelines
        self.logger = rclpy.logging.get_logger("muto_router")
        
    def route(self, action):
        pipeline = self.pipelines.get(action)
        if pipeline:
            pipeline.execute_pipeline()
        else:
            self.logger.warn(f"No pipeline found for action: {action}")
```

**Routing Logic**:

1. **Action Classification**: Analyze action type ("start", "stop", "apply")
2. **Pipeline Lookup**: Find configured pipeline for action
3. **Execution Dispatch**: Route to pipeline execution engine
4. **Error Handling**: Log routing failures and unsupported actions

**Supported Actions**:
- **`start`**: Deploy and launch new stack
- **`stop`/`kill`**: Stop and cleanup existing stack
- **`apply`**: Apply incremental changes to running stack

#### Router Integration

```python
# Router initialization in MutoComposer
def init_pipelines(self, pipeline_config):
    loaded_pipelines = {}
    
    for pipeline_item in pipeline_config:
        name = pipeline_item["name"]
        pipeline_spec = pipeline_item["pipeline"]
        compensation_spec = pipeline_item.get("compensation", None)
        
        pipeline = Pipeline(name, pipeline_spec, compensation_spec)
        loaded_pipelines[name] = pipeline
    
    self.pipelines = loaded_pipelines
    self.router = Router(self.pipelines)

# Action routing
def on_stack_callback(self, stack_msg):
    action = stack_msg.method  # "start", "stop", "apply"
    self.router.route(action)
```


### Safe Evaluator (`safe_evaluator.py`)

**Purpose**: Secure expression evaluation engine for pipeline conditional logic, preventing code injection while supporting complex boolean expressions.

**Security Features**:
- **Restricted AST**: Only safe AST node types allowed
- **No Function Calls**: Prevents arbitrary code execution
- **Controlled Context**: Limited variable access through controlled context
- **Type Safety**: Strict type checking for operations

#### SafeEvaluator Class

```python
class SafeEvaluator:
    operators = {
        ast.Eq: op.eq,
        ast.NotEq: op.ne,
        ast.Lt: op.lt,
        ast.LtE: op.le,
        ast.Gt: op.gt,
        ast.GtE: op.ge,
        ast.And: op.and_,
        ast.Or: op.or_,
        ast.Not: op.not_,
    }
    
    def __init__(self, context):
        self.context = context
```

**Key Methods**:

**`eval_expr(expr)`**
- Parse and evaluate conditional expressions
- Validate expression safety through AST analysis
- Return boolean results for pipeline decisions

**`_eval(node)`**
- Recursive AST node evaluation
- Handle comparison operations, logical operations, and attribute access
- Maintain security boundaries

#### Supported Expression Types

**Basic Comparisons**:
```python
# Context: {"step1": {"success": True}, "should_run": True}
evaluator = SafeEvaluator(context)

# Simple boolean
evaluator.eval_expr("should_run == True")  # True

# Attribute access
evaluator.eval_expr("step1.success == True")  # True

# Complex logical expressions
evaluator.eval_expr("step1.success == True and should_run == True")  # True
```

**Logical Operations**:
```python
# AND operations
"condition1 == True and condition2 == True"

# OR operations
"step1.success == True or step2.success == True"

# NOT operations
"not (step1.success == False)"
```

**Comparison Operations**:
```python
# Equality
"step_result.return_code == 0"

# String comparisons
"step_output.status == 'completed'"

# Numeric comparisons
"execution_time < 300"
```

#### Security Restrictions

The SafeEvaluator prevents dangerous operations:

```python
# BLOCKED: Function calls
"eval('malicious code')"  # Raises ValueError

# BLOCKED: Import statements
"import os"  # Raises ValueError

# BLOCKED: Attribute modification
"context.__dict__.clear()"  # Not supported

# ALLOWED: Safe comparisons
"step1.success == True"  # Safe evaluation
```

### Pipeline Schema (`schemas/pipeline_schema.py`)

**Purpose**: JSON Schema validation for pipeline configuration files, ensuring proper structure and required fields.

#### Schema Structure

```python
PIPELINE_SCHEMA = {
    "type": "object",
    "properties": {
        "pipelines": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "pipeline": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "sequence": {
                                    "type": "array",
                                    "items": {
                                        "type": "object",
                                        "properties": {
                                            "name": {"type": "string"},
                                            "service": {"type": "string"},
                                            "plugin": {"type": "string"},
                                            "condition": {"type": "string"},
                                        },
                                        "required": ["name", "service", "plugin"],
                                    },
                                }
                            },
                            "required": ["sequence"],
                        },
                    },
                    "compensation": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "service": {"type": "string"},
                                "plugin": {"type": "string"},
                            },
                            "required": ["service", "plugin"],
                        },
                    },
                },
                "required": ["name", "pipeline", "compensation"],
            },
        }
    },
    "required": ["pipelines"],
}
```

## Configuration Examples

### Basic Pipeline Configuration

```yaml
# pipeline.yaml
pipelines:
  - name: start
    pipeline:
      - sequence:
        - service: muto_compose
          plugin: ComposePlugin
          name: compose_step
        - service: muto_provision
          plugin: ProvisionPlugin
          name: provision_step
          condition: "should_run_provision == True"
        - service: muto_start_stack
          plugin: LaunchPlugin
          name: start_stack_step
          condition: "compose_step.success == True"
    compensation:
      - service: muto_kill_stack
        plugin: LaunchPlugin
```

### Advanced Pipeline with Complex Conditions

```yaml
pipelines:
  - name: conditional_deployment
    pipeline:
      - sequence:
        - service: muto_validate
          plugin: ValidationPlugin
          name: validate_step
        
        - service: muto_compose
          plugin: ComposePlugin
          name: compose_step
          condition: "validate_step.success == True"
        
        - service: muto_provision
          plugin: ProvisionPlugin
          name: provision_step
          condition: "compose_step.success == True and has_repository == True"
        
        - service: muto_start_stack
          plugin: LaunchPlugin
          name: launch_step
          condition: "compose_step.success == True and (provision_step.success == True or skip_provision == True)"
    
    compensation:
      - service: muto_cleanup
        plugin: CleanupPlugin
      - service: muto_kill_stack
        plugin: LaunchPlugin
```


## Integration Patterns

### Pipeline Integration with MutoComposer

```python
class MutoComposer(Node):
    def __init__(self):
        super().__init__("muto_composer")
        
        # Load and initialize pipelines
        pipeline_config_path = self.get_pipeline_config_path()
        pipeline_config = self.load_pipeline_config(pipeline_config_path)
        self.init_pipelines(pipeline_config["pipelines"])
        
        # Create router
        self.router = Router(self.pipelines)
    
    def pipeline_execute(self, method, context=None, next_manifest=None):
        """Execute pipeline with additional context."""
        pipeline = self.pipelines.get(method)
        if pipeline:
            pipeline.execute_pipeline(
                additional_context=context,
                next_manifest=next_manifest
            )
        else:
            self.get_logger().error(f"No pipeline found for method: {method}")
```

### Dynamic Pipeline Execution

```python
def determine_execution_path(self):
    """Determine which pipeline steps to execute based on stack characteristics."""
    
    # Analyze stack content
    has_archive_artifact = self.current_stack.get("metadata", {}).get("content_type") == "stack/archive"
    has_repository = self.current_stack.get("url") is not None
    
    # Set execution context
    execution_context = {
        "should_run_provision": has_archive_artifact or has_repository,
        "should_run_launch": True,
        "skip_provision": not (has_archive_artifact or has_repository),
        "has_repository": has_repository,
        "has_archive": has_archive_artifact
    }
    
    # Execute pipeline with context
    self.pipeline_execute(self.method, execution_context, self.current_stack)
```


This comprehensive workflow documentation provides complete coverage of Eclipse Muto's pipeline orchestration system, from basic concepts through advanced configuration and troubleshooting techniques.