# Eclipse Muto Reference Documentation

This section provides comprehensive technical reference documentation for all Eclipse Muto components, APIs, configuration options, and integration patterns.

## Overview

The Eclipse Muto reference documentation is organized by component to provide in-depth technical details for developers, system administrators, and advanced users. Each component section includes API references, configuration schemas, performance characteristics, and integration guidelines.

## Reference Structure

### Core Components
- **[Agent](./agent/readme.md)** - Main orchestration engine and ROS 2 integration layer
- **[Composer](./composer/readme.md)** - Stack composition and launch file generation engine  
- **[Core](./core/readme.md)** - Common utilities, data structures, and shared functionality
- **[Messages](./messages/readme.md)** - ROS 2 message and service type definitions

### Integration References
- **[Symphony Integration](./symphony/readme.md)** - Eclipse Symphony orchestration platform integration
- **[Ditto Integration](./ditto/readme.md)** - Eclipse Ditto digital twin platform integration
- **[MQTT Integration](./mqtt/readme.md)** - MQTT messaging protocol integration
- **[Container Integration](./container/readme.md)** - Docker/Podman container management

### Configuration References
- **[Configuration Schema](./configuration/readme.md)** - Complete configuration file reference
- **[Stack Format](./stack_format/readme.md)** - Stack definition JSON schema and examples
- **[Launch Integration](./launch/readme.md)** - ROS 2 launch system integration details

## Component Architecture Overview

Eclipse Muto follows a modular architecture where components communicate via ROS 2 topics and services:

```
┌─────────────────────────────────────────────────┐
│                Eclipse Symphony                 │
│              (Cloud Orchestration)              │
└─────────────┬───────────────────────────────────┘
              │ HTTPS/REST API
              │
┌─────────────▼───────────────────────────────────┐
│                 Muto Agent                      │
│        • Symphony Provider                      │
│        • Gateway                                │  
│        • Commands and Monitoring                │
└─────────────┬───────────────────────────────────┘
              │ ROS 2 Topics/Services
              │
┌─────────────▼───────────────────────────────────┐
│               Muto Composer                     │
│        • Launch File Generation                 │
│        • Parameter Resolution                   │
│        • Node Composition                       │
│        • Stack Management                       │  
│        • Lifecycle Control                      │
└─────────────┬───────────────────────────────────┘
              │ ROS 2 Launch API
              │
┌─────────────▼───────────────────────────────────┐
│                ROS 2 Nodes                      │
│        • User Applications                      │
│        • Sensor Drivers                         │
└─────────────────────────────────────────────────┘
```

## API Reference Overview

### Primary Interfaces

#### Stack Management API
```bash
# Topic: /muto/stack
# Message Type: muto_msgs/MutoAction
# Operations: start, stop, update, restart
```

#### Status Monitoring API  
```bash
# Topic: /muto/composer_response
# Message Type: muto_msgs/ComposerResponse
# Provides: deployment status, errors, lifecycle events
```

#### Service Configuration API
```bash
# Service: /muto/compose_plugin
# Service Type: muto_msgs/srv/ComposePlugin  
# Operations: status, validate, configure
```

### Message Types Summary

#### Core Messages
- `muto_msgs/MutoAction` - Stack deployment commands
- `muto_msgs/ComposerResponse` - Deployment status and results
- `muto_msgs/AgentStatus` - Agent health and capability information
- `muto_msgs/StackDefinition` - Complete stack configuration structure

#### Service Types  
- `muto_msgs/srv/ComposePlugin` - Composer plugin operations
- `muto_msgs/srv/StackManagement` - Advanced stack lifecycle management
- `muto_msgs/srv/ConfigurationValidation` - Configuration validation services


The **Muto Composer** is the heart of edge orchestration, managing ROS stack lifecycle and state reconciliation.

**Key Responsibilities:**
- Declarative stack management and deployment
- Continuous state monitoring and reconciliation
- Advanced pipeline execution with failure recovery
- Native ROS 2 launch system integration

**Core Modules:**
- **Stack Management**: Complete ROS stack lifecycle control
- **Pipeline System**: Workflow orchestration with compensation logic
- **Compose Plugin**: ROS launch system integration
- **Provision Plugin**: ROS launch system integration
- **Launch Plugin**: ROS launch system integration
- **Introspection Engine**: System analysis and monitoring

**Source Code**: [`src/composer/`](../../../src/composer/)

### Core (`src/core/` → [📖 Documentation](./core/README.md))

The **Core** component provides foundational services and utilities for managing the digital twins.

**Key Responsibilities:**
- Digital twin implementation and synchronization
- Common interfaces and base classes
- Configuration management and utilities
- Service integration frameworks

**Core Modules:**
- **Twin Services**: Digital twin management and synchronization
- **Common Interfaces**: Shared abstractions and patterns
- **Configuration System**: Centralized configuration handling

**Source Code**: [`src/core/`](../../../src/core/)

### Messages (`src/messages/` → [📖 Documentation](./messages/README.md))

The **Messages** component defines all ROS message types and service interfaces for system communication.

**Key Responsibilities:**
- ROS message type definitions for all components
- Service interface specifications
- Cross-component communication protocols
- API contract definitions

**Core Modules:**
- **Message Types**: ROS `.msg` files for pub/sub communication
- **Service Definitions**: ROS `.srv` files for request/response patterns
- **Build Configuration**: CMake and package configuration

**Source Code**: [`src/messages/`](../../../src/messages/)

## API Reference Overview

### Message-Based Communication

Eclipse Muto uses ROS 2 message patterns for all inter-component communication:

#### **Core Topics**
```python
# Stack management
/muto/stack                    # MutoAction messages
/muto/twin                     # Thing messages  
/muto/agent_to_gateway         # Gateway messages
/muto/gateway_to_agent         # Gateway messages
```

#### **Service Interfaces**
```python
# Component services
/muto/compose_plugin           # ComposePlugin.srv
/muto/launch_plugin            # LaunchPlugin.srv
/muto/provision_plugin         # ProvisionPlugin.srv
/muto/core_twin               # CoreTwin.srv
/muto/command_plugin          # CommandPlugin.srv
```

#### **Action Patterns**
```python
# Long-running operations
MutoAction                     # Stack deployment actions
StackManifest                  # Complete stack definitions
```

### Plugin Architecture

Eclipse Muto's plugin system enables extensibility across multiple dimensions:

#### **Protocol Plugins**
```python
class CustomProtocolPlugin(BaseProtocolPlugin):
    def handle_message(self, message: Any) -> bool:
        """Handle incoming protocol messages"""
        
    def send_response(self, response: Any) -> None:
        """Send response via protocol"""
```

#### **Compose Plugins**
```python
class CustomComposePlugin(BaseComposePlugin):
    def handle_custom(
        self, request: CustomComposePlugin.Request, response: CustomComposePlugin.Response
    ):
```


## Navigation Guide

### Getting Started with Reference Docs

1. **Component Overview**: Start with the component that interests you most
2. **API Reference**: Dive into specific interfaces and methods
3. **Integration Examples**: See how components work together
4. **Configuration Guide**: Understand customization options

### Component-Specific Navigation

- **Agent Deep Dive**: [Agent Reference](./agent/README.md)
- **Composer Architecture**: [Composer Reference](./composer/README.md)
- **Core Services**: [Core Reference](./core/README.md)
- **Message Protocols**: [Messages Reference](./messages/README.md)

### Cross-References

- **User Guide**: [How to use these components](../user_guide/readme.md)
- **Developer Guide**: [How to extend and modify](../developer_guide/readme.md)
- **Examples**: [Practical implementations](../user_guide/running_examples.md)

## Contributing to Documentation

Help improve Eclipse Muto's reference documentation:

- **API Documentation**: Keep interface docs synchronized with code
- **Examples**: Add practical examples and use cases  
- **Architecture Diagrams**: Create and maintain system diagrams
- **Performance Data**: Contribute benchmarks and measurements

---

## Support and Resources

- **Technical Questions**: Use [GitHub Discussions](https://github.com/eclipse-muto/muto/discussions)
- **Bug Reports**: Submit [GitHub Issues](https://github.com/eclipse-muto/muto/issues)
- **Feature Requests**: Propose enhancements via GitHub Issues
- **Community**: Join the Eclipse Muto developer community

**This reference documentation is your comprehensive guide to Eclipse Muto's technical implementation. Dive deep into the components that matter most for your use case!**