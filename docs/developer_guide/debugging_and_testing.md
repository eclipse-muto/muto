# Eclipse Muto - Debugging and Testing

This comprehensive guide covers debugging techniques, testing strategies, and troubleshooting methods for Eclipse Muto development.

## Overview

Effective debugging and testing are crucial for Eclipse Muto development. This guide provides tools, techniques, and best practices for:
- Debugging ROS 2 applications
- Unit and integration testing
- Performance profiling
- System-level troubleshooting

## Prerequisites

- ✅ [Development Environment](./building_from_source.md) completed
- ✅ Eclipse Muto built from source
- ✅ Familiarity with Python debugging and ROS 2 concepts

## Part 1: Debugging Strategies

### 1.1 Debug Build Configuration
```bash
# Build with debug symbols and verbose output
cd ~/muto_ws
colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS="-g -O0 -DDEBUG" \
    --event-handlers console_direct+

# Source the debug build
source install/setup.bash
```

### 1.2 Debugging with ROS2 Tools
Please follow the debugging instructions for [Robot Developer Extensions for ROS 2](https://ranchhandrobotics.com/rde-ros-2/)

## Part 2: ROS 2 Specific Debugging

### 2.1 ROS 2 Node Debugging
```bash
# Debug node discovery issues
ros2 daemon stop
ros2 daemon start
ros2 node list

# Debug topic communication
ros2 topic list
ros2 topic info /muto/stack
ros2 topic echo /muto/stack --once

# Debug service communication
ros2 service list | grep muto
ros2 service call /muto/compose_plugin muto_msgs/srv/ComposePlugin "{action: 'status'}"

# Debug parameter issues
ros2 param list | grep muto
ros2 param get /muto/muto_agent config_file
ros2 param dump /muto/muto_agent > agent_params.yaml
```

### 2.2 ROS 2 Launch Debugging
```bash
# Debug launch file issues
ros2 launch launch/muto.launch.py \
    vehicle_namespace:=org.eclipse.muto.test \
    vehicle_name:=test-robot-debug \
    enable_symphony:=true \
    log_level:=DEBUG


# Debug individual nodes in launch
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
ros2 launch launch/muto.launch.py \
    vehicle_namespace:=org.eclipse.muto.test \
    vehicle_name:=test-robot-debug \
    enable_symphony:=true \
    log_level:=DEBUG
```

### 2.3 ROS 2 Network Debugging
```bash
# Debug network discovery
ros2 multicast receive &
ros2 multicast send

# Debug domain issues
export ROS_DOMAIN_ID=42
ros2 node list

# Debug RMW issues
export RMW_IMPLEMENTATION=rmw_cyclonedx
ros2 doctor --report

# Debug QoS issues
ros2 topic info /muto/stack --verbose
```

## Summary

This debugging and testing guide provides comprehensive tools for Eclipse Muto development:

### 🔍 **Debugging Capabilities**
- Python debugging with PDB and remote debugging
- VS Code integration with launch configurations  
- ROS 2 specific debugging techniques
- Performance profiling and resource monitoring


### 📊 **Monitoring Tools**
- Real-time system resource monitoring
- Log analysis and pattern detection
- Automated debug report generation
- Health check and diagnostic scripts


These tools will significantly improve your development efficiency and help you quickly identify and resolve issues in Eclipse Muto!