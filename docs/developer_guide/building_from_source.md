# Eclipse Muto - Building from Source

This guide provides detailed instructions for building Eclipse Muto from source code, including dependency management, build optimization, and troubleshooting common build issues.

## Prerequisites

Before building from source, ensure you have completed the prereqs. You should have:

- ✅ Ubuntu with [ROS 2 installed](https://docs.ros.org/en/humble/Installation.html)
- ✅ Colcon build system and development tools
- ✅ Python 3.10+ with required packages
- ✅ Git configured for development
- **Dev Containers** If you do not want to contaminate your workstation use a development container such as [VsCode Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)


## Repository Structure Overview

Understanding the Eclipse Muto source structure helps with building and debugging:

```
muto/
├── src/                          # Source code
│   ├── agent/                    # Muto Agent component
│   │   ├── muto_agent/          # Python package
│   │   ├── test/                # Unit tests
│   │   └── package.xml          # ROS package definition
│   ├── composer/                # Muto Composer component
│   │   ├── muto_composer/       # Python package
│   │   ├── test/                # Unit tests
│   │   └── package.xml
│   ├── core/                    # Muto Core services
│   │   ├── muto_core/           # Python package
│   │   ├── test/                # Unit tests
│   │   └── package.xml
│   └── messages/                # ROS message definitions
│       ├── msg/                 # Message files (.msg)
│       ├── srv/                 # Service files (.srv)
│       ├── CMakeLists.txt       # CMake build configuration
│       └── package.xml
├── config/                      # Configuration files
├── launch/                      # ROS launch files
├── docs/                        # Documentation
└── newdocs/                     # New documentation structure
```

## Step 1: Workspace Setup

### 1.1 Create and Initialize Workspace
```bash
# Create ROS workspace
mkdir -p ~/muto_ws/src
cd ~/muto_ws

# Initialize workspace with basic setup
colcon build --symlink-install
source install/setup.bash
```

### 1.2 Clone Source Code
```bash
cd ~/muto_ws/src

# Clone the official repository
git clone https://github.com/eclipse-muto/muto.git

# Alternative: Clone your fork for development
# git clone https://github.com/YOUR_USERNAME/muto.git
# cd muto
# git remote add upstream https://github.com/eclipse-muto/muto.git
```

### 1.3 Verify Repository Structure
```bash
cd ~/muto_ws/src/muto

# Check repository structure
find . -name "package.xml" -type f
# Expected output:
# ./src/agent/package.xml
# ./src/composer/package.xml
# ./src/core/package.xml
# ./src/messages/package.xml

# Check Python packages
find . -name "__init__.py" -type f | head -10
```

## Step 2: Dependency Management

### 2.1 Install System Dependencies
```bash
cd ~/muto_ws

# Update rosdep database
rosdep update

# Install all package dependencies
rosdep install --from-paths src --ignore-src -r -y

# This installs ROS dependencies listed in package.xml files
```

### 2.2 Install Python Dependencies
```bash
# Install dependencies for agent
cd ~/muto_ws/src/muto/src/agent
if [ -f requirements.txt ]; then
    pip3 install --user -r requirements.txt
fi

# Install dependencies for composer
cd ~/muto_ws/src/muto/src/composer
if [ -f requirements.txt ]; then
    pip3 install --user -r requirements.txt
fi

# Install common dependencies manually
pip3 install --user \
    pyyaml \
    jsonschema \
    requests \
    websocket-client \
    paho-mqtt \
    aiohttp \
    asyncio-mqtt
```

### 2.3 Install Demo Dependencies (for testing)
```bash
# Install demo packages for examples and testing
sudo apt install -y \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-example-interfaces \
    ros-humble-rclcpp-components \
    ros-humble-composition-interfaces
```

## Step 3: Build Process

### 3.1 Clean Build (Recommended for First Build)
```bash
cd ~/muto_ws

# Clean any previous build artifacts
rm -rf build/ install/ log/

# Perform clean build
colcon build --symlink-install

# Source the built workspace
source install/setup.bash
```

### 3.2 Incremental Build (for Development)
```bash
cd ~/muto_ws

# Build only changed packages
colcon build --symlink-install --packages-up-to muto_agent muto_composer muto_core muto_msgs

# Build specific package only
colcon build --symlink-install --packages-select muto_agent

# Build with debug symbols
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### 3.3 Optimized Build (for Performance)
```bash
cd ~/muto_ws

# Parallel build with optimization
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

# Build with specific optimizations
colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-O3 -march=native" \
    -DCMAKE_C_FLAGS="-O3 -march=native"
```

## Step 4: Build Verification

### 4.1 Check Build Results
```bash
cd ~/muto_ws

# Verify all packages built successfully
colcon list --packages-up-to muto_agent muto_composer muto_core muto_msgs

# Check installed executables
find install/ -name "muto_*" -type f -executable
# Expected files:
# install/muto_agent/lib/muto_agent/muto_agent
# install/muto_composer/lib/muto_composer/muto_composer
# install/muto_core/lib/muto_core/muto_core

# Check Python packages
find install/ -name "muto_*" -type d | grep -E "site-packages|dist-packages"
```

### 4.2 Test Basic Functionality
```bash
# Source workspace
source ~/muto_ws/install/setup.bash

# Test package discovery
ros2 pkg list | grep muto
# Expected output:
# muto_agent
# muto_composer
# muto_core
# muto_msgs

# Test executable discovery
ros2 pkg executables muto_agent
ros2 pkg executables muto_composer
ros2 pkg executables muto_core

# Test message types
ros2 interface list | grep muto_msgs
```

### 4.3 Run Basic Components
```bash
# Test each component individually (in separate terminals)

# Terminal 1: Test core
ros2 run muto_core muto_core --ros-args --log-level debug

# Terminal 2: Test agent
ros2 run muto_agent muto_agent --ros-args --log-level debug

# Terminal 3: Test composer
ros2 run muto_composer muto_composer --ros-args --log-level debug

# Terminal 4: Check nodes are running
ros2 node list | grep muto
```

## Step 5: Build Customization

### 5.1 Development Build Configuration
```bash
# Create development build script
cat > ~/muto_ws/build_dev.sh << 'EOF'
#!/bin/bash
set -e

echo "Building Eclipse Muto for development..."

cd ~/muto_ws

# Clean build artifacts
rm -rf build/ install/ log/

# Build with debug symbols and development flags
colcon build \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --parallel-workers $(nproc) \
    --event-handlers console_direct+

# Source workspace
source install/setup.bash

echo "Development build complete!"
echo "Executables located in: install/*/lib/*/*"
EOF

chmod +x ~/muto_ws/build_dev.sh
```

### 5.2 Production Build Configuration
```bash
# Create production build script
cat > ~/muto_ws/build_prod.sh << 'EOF'
#!/bin/bash
set -e

echo "Building Eclipse Muto for production..."

cd ~/muto_ws

# Clean build artifacts
rm -rf build/ install/ log/

# Build with optimizations
colcon build \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG" \
    -DCMAKE_C_FLAGS="-O3 -DNDEBUG" \
    --parallel-workers $(nproc) \
    --event-handlers console_cohesion+

# Source workspace  
source install/setup.bash

echo "Production build complete!"
echo "Run 'source ~/muto_ws/install/setup.bash' to use built packages"
EOF

chmod +x ~/muto_ws/build_prod.sh
```

### 5.3 Testing Build Configuration
```bash
# Create testing build script
cat > ~/muto_ws/build_test.sh << 'EOF'
#!/bin/bash
set -e

echo "Building and testing Eclipse Muto..."

cd ~/muto_ws

# Build for testing
colcon build \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCOVERAGE=ON \
    --parallel-workers $(nproc)

# Source workspace
source install/setup.bash

# Run tests
echo "Running tests..."
colcon test \
    --event-handlers console_direct+ \
    --return-code-on-test-failure

# Show test results
colcon test-result --verbose

echo "Build and test complete!"
EOF

chmod +x ~/muto_ws/build_test.sh
```

## Step 6: Advanced Build Options

### 6.1 Cross-Compilation (for ARM64)
```bash
# Install cross-compilation tools
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Create cross-compilation build
cd ~/muto_ws

colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=../src/muto/cmake/aarch64-toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release
```

### 6.2 Static Analysis Build
```bash
# Build with static analysis tools
cd ~/muto_ws

# Install static analysis tools
sudo apt install -y cppcheck clang-tidy

# Build with static analysis
colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DENABLE_CPPCHECK=ON \
    -DENABLE_CLANG_TIDY=ON
```

### 6.3 Sanitizer Builds
```bash
# Build with AddressSanitizer
colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer" \
    -DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer"

# Build with ThreadSanitizer
colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS="-fsanitize=thread" \
    -DCMAKE_C_FLAGS="-fsanitize=thread"
```

## Step 7: Troubleshooting Build Issues

### 7.1 Common Build Errors

#### Error: Package not found
```bash
# Symptom: "Package 'xyz' not found"
# Solution: Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# For specific packages
sudo apt search ros-humble-package-name
sudo apt install ros-humble-package-name
```

#### Error: Python import failures
```bash
# Symptom: "ModuleNotFoundError: No module named 'xyz'"
# Solution: Install Python dependencies
pip3 install --user module-name

# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"
```

#### Error: CMake configuration fails
```bash
# Symptom: CMake errors during configuration
# Solution: Clean and reconfigure
rm -rf build/ install/
sudo apt update
sudo apt install cmake build-essential

colcon build --symlink-install --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

### 7.2 Build Performance Issues

#### Slow Build Times
```bash
# Use parallel builds
colcon build --parallel-workers $(nproc)

# Use ccache for faster rebuilds
sudo apt install ccache
export PATH="/usr/lib/ccache:$PATH"

# Build only changed packages
colcon build --packages-up-to changed_package
```

#### Memory Issues During Build
```bash
# Limit parallel workers to reduce memory usage
colcon build --parallel-workers 2

# Use swap if needed
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### 7.3 Dependency Issues

#### Missing ROS Dependencies
```bash
# Update rosdep database
sudo rosdep init  # Only if not already initialized
rosdep update

# Check for missing dependencies
rosdep check --from-paths src --ignore-src

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y --skip-keys="package_to_skip"
```

#### Python Environment Conflicts
```bash
# Check Python version
python3 --version

# Check pip version
pip3 --version

# Reset pip if corrupted
python3 -m pip install --user --upgrade pip setuptools wheel

# Use virtual environment if needed
python3 -m venv ~/muto_venv
source ~/muto_venv/bin/activate
pip install -r requirements.txt
```

## Step 8: Build Automation

### 8.1 Continuous Integration Build Script
```bash
# Create CI build script
cat > ~/muto_ws/ci_build.sh << 'EOF'
#!/bin/bash
set -euo pipefail

echo "=== Eclipse Muto CI Build ==="

# Environment setup
source /opt/ros/humble/setup.bash
cd ~/muto_ws

# Clean workspace
rm -rf build/ install/ log/

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_cohesion+ \
    --parallel-workers $(nproc)

# Test
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure

# Static analysis
if command -v flake8 >/dev/null; then
    echo "Running Python linting..."
    find src/muto -name "*.py" -exec flake8 {} +
fi

# Generate test results
colcon test-result --verbose > test_results.txt

echo "=== CI Build Complete ==="
EOF

chmod +x ~/muto_ws/ci_build.sh
```

### 8.2 Development Workflow Script
```bash
# Create development workflow script
cat > ~/muto_ws/dev_workflow.sh << 'EOF'
#!/bin/bash

case "$1" in
    "setup")
        echo "Setting up development environment..."
        source /opt/ros/humble/setup.bash
        cd ~/muto_ws
        rosdep install --from-paths src --ignore-src -r -y
        ;;
    "build")
        echo "Building for development..."
        cd ~/muto_ws
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
        source install/setup.bash
        ;;
    "test")
        echo "Running tests..."
        cd ~/muto_ws
        source install/setup.bash
        colcon test --event-handlers console_direct+
        ;;
    "clean")
        echo "Cleaning workspace..."
        cd ~/muto_ws
        rm -rf build/ install/ log/
        ;;
    "format")
        echo "Formatting code..."
        cd ~/muto_ws/src/muto
        find . -name "*.py" -exec black {} +
        find . -name "*.py" -exec isort {} +
        ;;
    *)
        echo "Usage: $0 {setup|build|test|clean|format}"
        exit 1
        ;;
esac
EOF

chmod +x ~/muto_ws/dev_workflow.sh

# Usage examples:
# ~/muto_ws/dev_workflow.sh setup
# ~/muto_ws/dev_workflow.sh build  
# ~/muto_ws/dev_workflow.sh test
```

## Step 9: Build Verification and Testing

### 9.1 Integration Test
```bash
# Create integration test script
cat > ~/muto_ws/integration_test.sh << 'EOF'
#!/bin/bash
set -e

echo "=== Eclipse Muto Integration Test ==="

cd ~/muto_ws
source install/setup.bash

# Start core in background
ros2 run muto_core muto_core &
CORE_PID=$!
sleep 2

# Start agent in background  
ros2 run muto_agent muto_agent &
AGENT_PID=$!
sleep 2

# Start composer in background
ros2 run muto_composer muto_composer &
COMPOSER_PID=$!
sleep 5

# Test basic functionality
echo "Testing node discovery..."
ros2 node list | grep muto

echo "Testing topic discovery..."
ros2 topic list | grep muto

# Deploy test stack
echo "Testing stack deployment..."
cat > /tmp/test_stack.json << 'STACK_EOF'
{
  "name": "Integration Test Stack",
  "stackId": "test:integration:v1.0",
  "node": [
    {
      "name": "test_talker",
      "pkg": "demo_nodes_cpp",
      "exec": "talker",
      "namespace": "/integration_test"
    }
  ]
}
STACK_EOF

# Deploy stack
ros2 topic pub --once /muto/stack muto_msgs/MutoAction \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'integration'}, 
   action: 'start', 
   stack: '$(cat /tmp/test_stack.json | jq -c .)'}"

sleep 5

# Check if test node is running
if ros2 node list | grep test_talker; then
    echo "✓ Integration test PASSED"
    TEST_RESULT=0
else
    echo "✗ Integration test FAILED"
    TEST_RESULT=1
fi

# Cleanup
kill $CORE_PID $AGENT_PID $COMPOSER_PID 2>/dev/null
rm -f /tmp/test_stack.json

exit $TEST_RESULT
EOF

chmod +x ~/muto_ws/integration_test.sh
```

## Conclusion

You now have Eclipse Muto built from source with multiple build configurations and verification scripts. Here's a summary of what you accomplished:

### ✅ Build Outputs
- **Executables**: `install/*/lib/*/*` - Ready to run Muto components
- **Python Packages**: `install/*/lib/python*/site-packages/muto_*` - Importable modules
- **Message Types**: `install/*/share/*/msg/` - ROS message definitions
- **Launch Files**: `install/*/share/*/launch/` - ROS launch configurations

### 🚀 Next Steps
1. **Run Components**: Use the built executables to start Muto services
2. **Deploy Stacks**: Test with sample stack configurations
3. **Develop Features**: Modify source code and rebuild incrementally
4. **Run Tests**: Execute the test suite to verify functionality
5. **Debug Issues**: Use debug builds with GDB or other debugging tools

### 🔧 Daily Development Workflow
```bash
# Quick development iteration
cd ~/muto_ws
~/muto_ws/dev_workflow.sh build    # Build changes
~/muto_ws/dev_workflow.sh test     # Run tests
~/muto_ws/integration_test.sh      # Verify integration
```

Your Eclipse Muto build is now ready for development, testing, and deployment!