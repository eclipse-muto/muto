# Eclipse Muto - Troubleshooting Guide

This comprehensive troubleshooting guide helps you diagnose and resolve common issues when working with Eclipse Muto across different deployment scenarios.

## Table of Contents
- [Installation Issues](#installation-issues)
- [Runtime Problems](#runtime-problems)
- [Stack Deployment Issues](#stack-deployment-issues)
- [Symphony Integration Problems](#symphony-integration-problems)
- [Performance Issues](#performance-issues)
- [Network and Connectivity](#network-and-connectivity)
- [Container-Specific Issues](#container-specific-issues)
- [Development Environment Issues](#development-environment-issues)

---

## Installation Issues

### ROS 2 Installation Problems

#### Issue: `ros2: command not found`
**Symptoms**: ROS 2 commands not recognized in terminal
**Diagnosis**:
```bash
# Check if ROS 2 is installed
ls /opt/ros/
# Should show 'humble' directory

# Check PATH
echo $PATH | grep ros
```
**Solutions**:
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to bashrc permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# If ROS 2 not installed, install it
sudo apt update
sudo apt install ros-humble-desktop
```

#### Issue: `colcon: command not found`
**Symptoms**: Build system not available
**Solutions**:
```bash
# Install colcon
sudo apt install python3-colcon-common-extensions

# Verify installation
colcon --help
```

#### Issue: Package dependencies not found
**Symptoms**: Missing ROS packages during build
**Solutions**:
```bash
# Update package database
sudo apt update

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# For specific packages
sudo apt install ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py
```

### Muto Build Issues

#### Issue: Build fails with import errors
**Symptoms**: Python import errors during colcon build
**Diagnosis**:
```bash
# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"

# Check for missing packages
python3 -c "import rclpy, yaml, requests"
```
**Solutions**:
```bash
# Install missing Python packages
pip3 install --user rclpy pyyaml requests paho-mqtt

# For development packages
pip3 install --user pytest pytest-cov black flake8
```

#### Issue: Workspace build fails
**Symptoms**: `colcon build` returns errors
**Diagnosis**:
```bash
# Build with verbose output
colcon build --event-handlers console_direct+

# Check build logs
cat log/latest_build/*/stdout_stderr.log
```
**Solutions**:
```bash
# Clean workspace
rm -rf build install log

# Build individual packages
colcon build --packages-select muto_msgs
colcon build --packages-select muto_core
colcon build --packages-select muto_agent
colcon build --packages-select muto_composer

# Source after each successful build
source install/setup.bash
```

---

## Runtime Problems

### Muto Components Not Starting

#### Issue: Nodes fail to start
**Symptoms**: `ros2 node list` doesn't show Muto nodes
**Diagnosis**:
```bash
# Check ROS environment
printenv | grep ROS

# Test basic ROS functionality
ros2 topic list

# Check configuration
cat $MUTO_CONFIG_PATH
```
**Solutions**:
```bash
# Ensure workspace is sourced
source ~/muto_ws/install/setup.bash

# Check configuration syntax
python3 -c "import yaml; yaml.safe_load(open('$MUTO_CONFIG_PATH'))"

# Start with minimal config
ros2 run muto_agent muto_agent --ros-args -p config_file:=/dev/null
```

#### Issue: Permission denied errors
**Symptoms**: Components fail with permission errors
**Solutions**:
```bash
# Fix file permissions
chmod +r $MUTO_CONFIG_PATH

# Fix Python package permissions
pip3 install --user --upgrade --force-reinstall pyyaml

# Check ROS workspace permissions
ls -la ~/muto_ws/install/
```


---

## Symphony Integration Problems

### Authentication Issues

#### Issue: Symphony authentication fails
**Symptoms**: HTTP 401/403 errors in logs
**Diagnosis**:
```bash
# Test Symphony connectivity
curl -I http://localhost:8082/v1alpha2/greetings

# Check authentication
curl -X POST \
  -H "Content-Type: application/json" \
  -d '{"username":"admin","password":""}' \
  http://localhost:8082/v1alpha2/users/auth
```
**Solutions**:
```bash
# Update Muto configuration
nano $MUTO_CONFIG_PATH
# Set correct Symphony URL and credentials

# Restart Symphony if needed
cd ~/symphony
docker-compose restart

# Verify Symphony provider is enabled
ros2 param get /muto/muto_agent symphony_provider.enabled
```

#### Issue: Symphony services not available
**Symptoms**: Connection refused errors
**Solutions**:
```bash
# Check Symphony containers
docker-compose ps

# Check ports
netstat -tuln | grep 8082

# Restart Symphony
docker-compose down
docker-compose up -d

# Wait for services to be ready
sleep 30
curl http://localhost:8082/v1alpha2/greetings
```


## Getting Help

When troubleshooting doesn't resolve your issue:

1. **📋 Collect Information**: Use the diagnostic scripts above
2. **🔍 Search Issues**: Check [GitHub Issues](https://github.com/eclipse-muto/muto/issues)
3. **💬 Community**: Ask in [GitHub Discussions](https://github.com/eclipse-muto/muto/discussions)
4. **🐛 Report Bug**: Create detailed issue with logs and reproduction steps
5. **📖 Documentation**: Review [User Guide](./readme.md) and [Reference](../reference/readme.md)

### Issue Report Template

When reporting issues, include:

```
**Environment:**
- OS: Ubuntu 22.04
- ROS 2: Humble
- Muto Version: [commit hash]
- Deployment: Native/Container

**Problem Description:**
[Detailed description]

**Steps to Reproduce:**
1. [First step]
2. [Second step]
3. [Issue occurs]

**Expected Behavior:**
[What should happen]

**Actual Behavior:**
[What actually happens]

**Logs:**
[Attach diagnostic logs]

**Configuration:**
[Attach muto_config.yaml]
```

This troubleshooting guide covers the most common issues encountered when working with Eclipse Muto. Keep it handy for quick problem resolution!