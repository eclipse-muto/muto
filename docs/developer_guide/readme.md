# Eclipse Muto - Developer Guide

Welcome to the Eclipse Muto Developer Guide! This comprehensive guide provides everything you need to contribute to, extend, and develop with Eclipse Muto.

## What You'll Find Here

This developer guide covers all aspects of Eclipse Muto development:

### 🛠️ Development Setup
- **[Building from Source](./building_from_source.md)** - Build Muto components from source code
- **[Running from Source](./running_from_source.md)** - Execute and test Muto components locally

### 🧪 Testing and Quality
- **[Debugging and Testing](./debugging_and_testing.md)** - Testing frameworks, debugging tools, and quality assurance

### 🤝 Contributing
- **[Code Contribution Guidelines](./code_contribution_guidelines.md)** - How to contribute code, documentation, and improvements

## Who Should Use This Guide

This guide is designed for:

- **Software Developers**: Contributing to Eclipse Muto core components
- **Platform Engineers**: Extending Muto for custom use cases
- **Researchers**: Understanding and modifying Muto's architecture
- **DevOps Engineers**: Custom deployment and integration scenarios
- **Open Source Contributors**: Contributing to the Eclipse Muto project

## Prerequisites

Before diving into development, you should have:

- **Strong ROS 2 Experience**: Understanding of ROS 2 concepts, launch systems, and node development
- **Python Proficiency**: Most Muto components are written in Python 3.8+
- **Linux Expertise**: Development is primarily on Ubuntu 20.04/22.04
- **Git/GitHub Skills**: Version control and collaborative development
- **Container Knowledge**: Docker/Podman for testing and deployment
- **Networking Basics**: MQTT, HTTP protocols for distributed systems

## Development Environment Options

### 🐳 Development Containers (Recommended)
**Best for: Consistent environment, VS Code integration, quick setup**

- Pre-configured development environment
- All dependencies included
- VS Code devcontainer integration
- Consistent across team members
- Easy CI/CD integration

### 🔧 Native Development Setup
**Best for: Performance, system integration, advanced debugging**

- Direct system access and performance
- Full debugging capabilities
- Custom system configurations
- Advanced development workflows
- Integration with system tools


## Architecture Overview for Developers

Understanding Muto's architecture is crucial for effective development:

### Core Components

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Agent         │    │   Composer       │    │   Core          │
│   - Message     │────┤   - Stack Mgmt   │────┤   - Twin        │
│   - MQTT        │    │   - Pipelines    │    │   - Services    │
│   - Symphony    │    │   - Plugins      │    │   - Utils       │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌──────────────────┐
                    │   Messages       │
                    │   - ROS Types    │
                    │   - Services     │
                    │   - Interfaces   │
                    └──────────────────┘
```

### Key Development Patterns

#### Plugin Architecture
Muto uses extensive plugin patterns for extensibility:
- **Protocol Plugins**: MQTT, HTTP, future Zenoh/uProtocol
- **Compose Plugins**: Stack composition and deployment
- **Launch Plugins**: ROS launch system integration
- **Command Plugins**: Command execution and routing

#### Message-Driven Architecture
All communication uses ROS 2 message patterns:
- **Publisher/Subscriber**: Asynchronous communication
- **Service/Client**: Synchronous request/response
- **Action/Client**: Long-running operations with feedback

#### State Management
Declarative state management with reconciliation loops:
- **Desired State**: JSON/binary stack definitions
- **Current State**: Real-time system status
- **Reconciliation**: Continuous convergence processes

## Development Workflows

### Standard Development Cycle

1. **Setup Environment**: Choose development environment option
2. **Clone and Build**: Get source code and build workspace
3. **Create Feature Branch**: Use Git flow for feature development
4. **Implement Changes**: Follow coding standards and patterns
5. **Write Tests**: Comprehensive unit and integration tests
6. **Run Quality Checks**: Linting, static analysis, security scans
7. **Test Integration**: End-to-end testing with real systems
8. **Create Pull Request**: Follow contribution guidelines
9. **Code Review**: Collaborate with maintainers and community
10. **Merge and Deploy**: Integration into main branch

### Testing Strategy

#### Unit Testing
- **Framework**: `colcon test` for all components
- **Coverage**: Minimum 80% code coverage required
- **Mocking**: Mock external dependencies and ROS interfaces
- **Isolation**: Each test should be independent and repeatable


### Code Quality Standards

#### Python Code Style
- **PEP 8**: Standard Python style guide compliance
- **Type Hints**: Full type annotation for all functions
- **Documentation**: Comprehensive docstrings and comments
- **Error Handling**: Robust exception handling and logging

#### ROS 2 Best Practices
- **Node Lifecycle**: Proper initialization and cleanup
- **Parameter Handling**: Use ROS parameters for configuration
- **Topic Design**: Efficient and scalable topic structures
- **Service Design**: Well-defined service interfaces

#### Security Considerations
- **Input Validation**: Validate all external inputs
- **Authentication**: Secure communication protocols
- **Authorization**: Role-based access controls
- **Encryption**: End-to-end encryption for sensitive data


### Getting Started with Contributions

1. **Read the Guidelines**: Start with [Code Contribution Guidelines](./code_contribution_guidelines.md)
2. **Set Up Environment**: Follow [Development Environment Setup](./development_environment_setup.md)
3. **Find an Issue**: Browse [GitHub Issues](https://github.com/eclipse-muto/muto/issues) for beginner-friendly tasks
4. **Join the Community**: Engage with developers on GitHub discussions
5. **Start Small**: Begin with documentation or small bug fixes
6. **Ask Questions**: Don't hesitate to ask for help and guidance

## Development Tools and Resources

### Recommended Tools
- **VS Code**: Primary development environment with ROS extensions
- **Git**: Version control with GitHub integration
- **Docker**: Container development and testing
- **ROS 2 Tools**: `colcon`, `rosdep`, `ros2` CLI tools
- **Python Tools**: `pytest`, `black`, `mypy`, `flake8`

### Useful Resources
- **ROS 2 Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **Eclipse Foundation**: [https://www.eclipse.org/](https://www.eclipse.org/)
- **Muto GitHub**: [https://github.com/eclipse-muto/muto](https://github.com/eclipse-muto/muto)
- **Symphony Project**: [https://github.com/eclipse-symphony/symphony](https://github.com/eclipse-symphony/symphony)

### Community Channels
- **GitHub Discussions**: Primary community forum
- **Issue Tracker**: Bug reports and feature requests
- **Pull Requests**: Code review and collaboration
- **Eclipse Muto Committers**: Direct contact for complex issues

---

## Quick Start for Developers

Ready to start developing? Here's your path:

1. **🛠️ [Set Up Environment](./development_environment_setup.md)** - Get your development environment ready
2. **🔧 [Build from Source](./building_from_source.md)** - Build and understand the codebase
3. **🧪 [Run Tests](./debugging_and_testing.md)** - Verify your setup and run the test suite
4. **📋 [Choose a Task](./code_contribution_guidelines.md#finding-tasks-to-work-on)** - Find something to work on
5. **🤝 [Make Your First PR](./code_contribution_guidelines.md#pull-request-process)** - Contribute your first change

**Welcome to the Eclipse Muto developer community!** We're excited to have you contribute to the future of robotic orchestration.