# Eclipse Muto - Examples and Tutorials

Welcome to the Eclipse Muto examples collection! This comprehensive guide provides hands-on examples, tutorials, and sample implementations to help you master Eclipse Muto's capabilities.

## Overview of Examples

This examples collection demonstrates Eclipse Muto's versatility across different robotics scenarios and use cases. Each example includes complete source code, configuration files, and step-by-step instructions.

### 📚 What You'll Learn

- **Declarative Stack Definitions**: How to create JSON-based ROS stack configurations
- **Remote Deployment**: Deploy stacks from cloud orchestration platforms
- **State Management**: Understand how Muto maintains and reconciles system state
- **Fleet Operations**: Manage multiple devices with consistent configurations
- **Integration Patterns**: Connect with Eclipse Symphony and other platforms
- **Advanced Features**: Composable nodes, parameter management, and pipeline orchestration

### 🎯 Example Categories

#### Basic Examples
- **Simple Talker-Listener**: Classic ROS 2 communication patterns
- **Parameter Configuration**: Dynamic parameter management
- **Node Lifecycle**: Start, stop, and reload operations
- **Service Integration**: ROS service-based interactions

#### Intermediate Examples  
- **Navigation Stack**: Mobile robot navigation with mapping and planning
- **Perception Pipeline**: Camera and sensor data processing
- **Multi-Robot Coordination**: Coordinated operations across multiple devices
- **Custom Message Types**: Custom ROS messages and service definitions

#### Advanced Examples
- **Industrial Automation**: Factory robot coordination and task management
- **Autonomous Vehicles**: Fleet management for autonomous driving systems
- **Edge Computing Integration**: AI/ML workloads with robotics systems
- **Cloud Orchestration**: Full Eclipse Symphony integration scenarios

#### Real-World Scenarios
- **AprilTag Detection**: Computer vision and marker detection systems
- **Warehouse Robotics**: Logistics and inventory management robots
- **Service Robots**: Customer interaction and task-oriented robots
- **Research Platforms**: Flexible research and development scenarios

## Step-by-Step Guide for Each Example

Each example follows a consistent structure for easy learning:

### 1. **Overview and Objectives**
- Clear description of what the example demonstrates
- Learning objectives and key concepts covered
- Prerequisites and required knowledge

### 2. **System Requirements**
- Hardware requirements (if applicable)
- Software dependencies and versions
- Network and connectivity requirements

### 3. **Configuration Files**
- Complete Muto stack definitions (JSON format)
- ROS parameter files and launch configurations
- Environment-specific settings

### 4. **Deployment Instructions**
- Step-by-step deployment procedures
- Command-line instructions and scripts
- Verification and validation steps

### 5. **Expected Output**
- Sample output and log messages
- Performance metrics and benchmarks
- Troubleshooting common issues

### 6. **Customization Options**
- How to modify the example for your needs
- Parameter tuning and optimization tips
- Extension and integration possibilities

## Expected Output

### Successful Deployment Indicators

When examples are running correctly, you should see:

#### **System Status**
```bash
# All Muto nodes running
$ ros2 node list | grep muto
/muto/agent
/muto/muto_composer  
/muto/compose_plugin
/muto/launch_plugin
/muto/core_twin

# Stack deployment confirmation
$ ros2 topic echo /muto/stack --once
header:
  stamp: {sec: 1634567890, nanosec: 123456789}
  frame_id: 'muto_system'
action: 'start'
stack: '{"name": "Example Stack", "stackId": "example:stack:v1.0"}'
```

#### **Application Nodes**
```bash
# Example: Talker-Listener nodes
$ ros2 node list | grep -E "(talker|listener)"
/demo/talker
/demo/listener

# Message flow verification
$ ros2 topic echo /chatter
data: "Hello World: 1"
---
data: "Hello World: 2"  
---
```

#### **Digital Twin Synchronization**
```bash
# Twin state updates
$ ros2 topic echo /muto/twin --once
namespace: "org.eclipse.muto.demo"
name: "example-robot"
type: "mobile_robot"
state: "running"
stack_id: "example:stack:v1.0"
```

### Performance Metrics

#### **Resource Utilization**
- **CPU Usage**: Typically 5-15% for basic examples
- **Memory Usage**: 100-500 MB depending on stack complexity  
- **Network Traffic**: Minimal for local examples, varies for cloud integration

#### **Latency Measurements**
- **Stack Deployment**: 2-10 seconds for simple stacks
- **State Reconciliation**: Sub-second for configuration changes
- **Message Routing**: <10ms for local ROS communication

## Modifying and Extending Examples

### Customization Guidelines

#### **Stack Configuration**
Modify JSON stack definitions to add or change components:

```json
{
  "name": "Custom Navigation Stack",
  "stackId": "custom:navigation:v1.0",
  "param": [
    {"name": "robot_radius", "value": 0.5},
    {"name": "max_velocity", "value": 2.0}
  ],
  "node": [
    {
      "name": "map_server",
      "pkg": "nav2_map_server",
      "exec": "map_server",
      "param": [
        {"name": "yaml_filename", "value": "/maps/custom_map.yaml"}
      ]
    },
    {
      "name": "planner",
      "pkg": "nav2_planner",
      "exec": "planner_server",
      "namespace": "/navigation",
      "param": [
        {"name": "plugin", "value": "nav2_navfn_planner/NavfnPlanner"}
      ]
    }
  ]
}
```

#### **Parameter Tuning**
Adjust system parameters for your specific requirements:

```yaml
# config/custom_config.yaml
/**:
  ros__parameters:
    # Device-specific settings
    vehicle_namespace: "org.company.robots"
    vehicle_name: "custom-robot-001"
    
    # Performance tuning
    stack_reconciliation_rate: 10.0  # Hz
    state_monitoring_enabled: true
    
    # Integration settings
    symphony_enabled: true
    symphony_auto_register: true
```

#### **Custom Message Types**
Define custom message types for specific applications:

```python
# Custom message definition
# msg/CustomRobotState.msg
std_msgs/Header header
string robot_id
geometry_msgs/Pose current_pose
string current_task
float32 battery_level
bool is_autonomous
```

### Extension Patterns

#### **Adding New Protocols**
Extend Muto's protocol support:

```python
# Custom protocol plugin example
class CustomProtocolHandler(BaseProtocolHandler):
    def __init__(self):
        super().__init__()
        self.protocol_name = "custom_protocol"
    
    def handle_message(self, message):
        # Custom message processing logic
        pass
    
    def send_response(self, response):
        # Custom response handling
        pass
```

#### **Custom Compose Plugins**
Create specialized composition plugins:

```python
# Custom compose plugin example  
class CustomComposePlugin(BaseComposePlugin):
    def compose_stack(self, stack_definition):
        # Custom stack composition logic
        return composed_stack
    
    def validate_stack(self, stack_definition):
        # Custom validation rules
        return validation_result
```

## Available Sample Implementations

### Core Examples (Included)

#### 1. **Talker-Listener Demo** (`docs/samples/talker-listener/`)
**Description**: Classic ROS 2 pub/sub communication
- **Demonstrates**: Basic stack deployment, node configuration
- **Stack Format**: JSON with simple node definitions
- **Complexity**: Beginner
- **Runtime**: 5 minutes setup

#### 2. **AprilTag Robot** (`docs/samples/april-tag-robot/`)  
**Description**: Computer vision with marker detection
- **Demonstrates**: Perception pipeline, image processing
- **Stack Format**: JSON with composable nodes
- **Complexity**: Intermediate
- **Runtime**: 15 minutes setup

#### 3. **Symphony Integration** (`docs/samples/symphony/`)
**Description**: Full Eclipse Symphony orchestration
- **Demonstrates**: Cloud deployment, fleet management
- **Stack Format**: Symphony solutions and instances
- **Complexity**: Advanced  
- **Runtime**: 30 minutes setup

### Extended Examples (Available Online)

#### Industrial Automation Suite
**Repository**: `eclipse-muto/industrial-examples`
- Factory robot coordination
- Quality control systems
- Inventory management
- Production line integration

#### Autonomous Vehicle Platform  
**Repository**: `eclipse-muto/vehicle-examples`
- Fleet management systems
- Navigation and planning
- Sensor fusion pipelines
- Safety and monitoring

#### Service Robot Applications
**Repository**: `eclipse-muto/service-examples`
- Customer interaction robots
- Delivery and logistics
- Cleaning and maintenance
- Entertainment and education

## Getting Started with Examples

### Quick Start Checklist

1. **✅ Prerequisites Met**
   - Eclipse Muto installed and running
   - ROS 2 workspace configured
   - Required packages installed

2. **✅ Choose Your Example**
   - Start with [Talker-Listener](./samples/talker-listener/README.md) for beginners
   - Try [AprilTag Robot](./samples/april-tag-robot/README.md) for vision
   - Explore [Symphony Integration](./samples/symphony/README.md) for orchestration

3. **✅ Follow the Guide**
   - Read example overview and objectives
   - Check system requirements
   - Follow step-by-step instructions
   - Verify expected output

4. **✅ Experiment and Learn**
   - Modify configurations and parameters
   - Try different deployment methods
   - Extend with your own features

### Navigation Guide

- **New to Muto**: Start with [Talker-Listener Example](./samples/talker-listener/README.md)
- **Computer Vision**: Explore [AprilTag Detection](./samples/april-tag-robot/README.md)  
- **Cloud Integration**: Learn [Symphony Orchestration](./samples/symphony/README.md)
- **Custom Development**: Review modification and extension patterns above

## Community Examples

Share your own examples and learn from the community:

- **Contributing**: Submit your examples via pull requests
- **Showcasing**: Share your implementations in GitHub discussions
- **Learning**: Browse community examples for inspiration
- **Supporting**: Help others with questions and issues

---

## Next Steps

🚀 **Start with Examples**: Choose your first example from the [samples directory](./samples/)

📚 **Learn More**: Dive deeper with the [User Guide](../USER_GUIDE/README.md)

🛠️ **Develop**: Create your own solutions with the [Developer Guide](../DEVELOPER_GUIDE/README.md)

🔍 **Reference**: Technical details in [Reference Documentation](../REFERENCE/README.md)

**Happy exploring!** These examples will give you hands-on experience with Eclipse Muto's powerful orchestration capabilities.