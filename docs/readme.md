# Eclipse Muto

<img src="./images/muto.png" alt="Eclipse Muto Modules" title="Eclipse Muto Modules" width="30%" style="display: block; margin: auto; width: 30%;">

## Overview

**Eclipse Muto** is an open-source, declarative orchestrator for managing ROS (Robot Operating System) software stacks on edge devices. It is an approach to robotics infrastructure management, enabling cloud-native orchestration of distributed ROS systems while maintaining the real-time performance and deterministic behavior required for robotic applications.

It is designed to provide an an adaptive framework and that allows for dynamically composable model-driven software stacks for ROS during runtime. This allows Muto to modify the software solution to address some of the runtime adaptivity challenges in autonomous and robotic platforms based on policies or current context.


## What Makes Muto Unique

Eclipse Muto bridges the gap between traditional ROS development and modern cloud-native orchestration by providing a **model-based approach** to robot software management. Unlike conventional deployment methods that require manual intervention and physical access to devices, Muto enables declarative stack definitions that can be managed, versioned, and deployed remotely across entire robot fleets.

It is Adaptive; Muto is a context aware software solution to address some of the runtime adaptivity challenges in autonomous and robotic platforms. Adaptive Muto stacks support connected architectures for autonomous and software-defined vehicles, including supporting cloud-based apps and services that can dynamically change.  

It is Extensible; Muto is built on an extensible architecture. Similar to plugins and the extension points that are foundational to Eclipse extensibility. many of Muto's functionality are implemented as plugins that you can easily replace and modify for your requirements. You can add new ROS nodes for controlling, monitoring, composing and adapting vehicle behavior as plugins. 


## Key Features

- **Declarative ROS Orchestration**: Transform traditional imperative ROS launch systems into declarative, serializable formats
- **Remote Fleet Management**: Centralized control and deployment of software stacks across robot fleets
- **Model-Based Approach**: Complete ROS logic preservation while enabling remote management at scale
- **Multi-Protocol Support**: MQTT, HTTP, and future support for Zenoh and uProtocol
- **Digital Twin Integration**: Eclipse Ditto integration for device representation and synchronization
- **Cloud Orchestration**: Eclipse Symphony integration for scalable, policy-driven fleet management
- **Container Support**: Docker/Podman deployment for consistent environments across architectures (planned)
- **Extensible Plugin Architecture**: Customizable components for various use cases

## Repository Structure

```
├── README.md                     # This file
├── docs/                      # Complete documentation suite
│   ├── project_overview.md      # Detailed project overview and architecture
│   ├── user_guide/               # User documentation and guides
│   ├── developer_guide/          # Developer documentation
│   ├── examples/                 # Examples and sample implementations
│   └── reference/                # Technical reference documentation
├── src/                          # Source code
│   ├── agent/                    # Muto Agent - device communication and orchestration
│   ├── composer/                 # Muto Composer - ROS stack management and reconciliation
│   ├── core/                     # Core services and digital twin implementation
│   └── messages/                 # ROS message definitions and service interfaces
├── config/                       # Muto Configuration example files
└── launch/                       # Muto ROS launch  example files
```

All folder under the src/ folder are [GIT modules](https://git-scm.com/docs/gitmodules) that refer to other repositories.

## Quick Links


### 🚀 Getting Started
- **[🐳 Quick Start](./user_guide/quick_start.md)** - Fast deployment with pre-built images (recommended)
- **[🔧 Source Build Quick Start](./developer_guide/building_from_source.md)** - Build from source for maximum flexibility, including setup for  Containerized development environment

### 📚 Documentation
- **[📖 User Guide](./user_guide/readme.md)** - User documentation on how to configure and run Muto
- **[🔧 Developer Guide](./developer_guide/readme.md)** - Development and contribution guide
- **[📚 Reference Documentation](./reference/readme.md)** - Technical reference for all components

### 📝 Examples
- **[🎯 Examples Overview](./examples/readme.md)** - Muto examples and demonstration tutorials

## Target Use Cases

- **Autonomous Vehicle Fleets**: Centralized management of software stacks across vehicle fleets
- **Industrial Robotics**: Coordinated updates and configuration management for factory robots  
- **Service Robotics**: Remote deployment and monitoring of service robot applications
- **Research & Development**: Rapid prototyping and deployment of experimental ROS stacks
- **Edge Computing**: Integration of robotic systems with edge computing infrastructure

## License

Eclipse Muto is licensed under the [Eclipse Public License 2.0](../LICENSE).

## Contributing

We welcome contributions to Eclipse Muto! Please see our [Developer Guide](./developer_guide/readme.md) for detailed information on how to contribute, including:

## Community

- **GitHub Repository**: [https://github.com/eclipse-muto/muto](https://github.com/eclipse-muto/muto)
- **Eclipse Foundation Project Page**: [https://projects.eclipse.org/projects/automotive.muto](https://projects.eclipse.org/projects/automotive.muto)
- **Issue Tracker**: [GitHub Issues](https://github.com/eclipse-muto/muto/issues)

---

**Ready to get started?** Choose your preferred deployment approach from the links above!