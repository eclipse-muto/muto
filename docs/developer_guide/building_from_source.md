# Eclipse Muto - Building from Source

This guide provides detailed instructions for building Eclipse Muto from source code, including dependency management, build optimization, and troubleshooting common build issues.

## Prerequisites

Before building from source, ensure you have completed the prereqs. You should have:

- ✅ A supported OS / ROS 2 combination with [ROS 2 installed](https://docs.ros.org/en/humble/Installation.html):

  | ROS 2 distro | Ubuntu | Python |
  |---|---|---|
  | Humble | 22.04 | 3.10 |
  | Jazzy | 24.04 | 3.12 |

  All commands in this guide work on both distros; they use the `ROS_DISTRO` environment variable (see Step 1.0).
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
└── docs/                        # Documentation
```

## Step 1: Workspace Setup

### 1.0 Select Your ROS 2 Distribution
```bash
# Pick the distro installed on your machine: humble (Ubuntu 22.04) or jazzy (Ubuntu 24.04)
export ROS_DISTRO=humble   # or: jazzy

# Sourcing the ROS 2 setup script also sets ROS_DISTRO, so you can add this to ~/.bashrc
source /opt/ros/$ROS_DISTRO/setup.bash
```

The remaining steps assume `ROS_DISTRO` is set in your shell. The helper scripts created in Steps 5 and 8 default to `humble` when it is not set.

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
git clone --recurse-submodules  https://github.com/eclipse-muto/muto.git

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
# Expected output (order may vary):
# ./src/agent/package.xml
# ./src/agent/muto_paho_mqtt_vendor/package.xml   <- nested in the agent, needs --base-paths (see Step 3)
# ./src/composer/package.xml
# ./src/core/package.xml
# ./src/messages/package.xml
# ./src/eclipse_muto/package.xml                   <- meta package
# ./docs/samples/talker-listener/sample-stack/src/muto_listener/package.xml
# ./docs/samples/talker-listener/sample-stack/src/muto_talker/package.xml

# Check Python packages
find . -name "__init__.py" -type f | head -10
```

## Step 2: Dependency Management

### 2.1 Install System Dependencies
```bash
cd ~/muto_ws

# Refresh the apt package index (required on fresh systems / dev containers, otherwise
# rosdep and apt fail with "Unable to locate package")
sudo apt update

# Update rosdep database
rosdep update

# Install all package dependencies
rosdep install --from-paths src src/muto/src/agent/muto_paho_mqtt_vendor --ignore-src -r -y

# This installs ROS dependencies listed in package.xml files
```

### 2.2 Install Python Dependencies
```bash
# pip options differ per distro:
#  - Humble / Ubuntu 22.04: the system pip is too old to build the hatchling-based symphony-sdk, upgrade it first
#  - Jazzy  / Ubuntu 24.04: the system Python is "externally managed" (PEP 668), so --user installs need
#    --break-system-packages (or use a virtual environment, see 7.3)
if [ "$ROS_DISTRO" = "humble" ]; then
    python3 -m pip install --user --upgrade pip
    PIP_FLAGS="--user"
else
    PIP_FLAGS="--user --break-system-packages"
fi

# Install the Symphony SDK bundled with the agent (required by muto_agent)
python3 -m pip install $PIP_FLAGS ~/muto_ws/src/muto/src/agent/symphony-sdk-python

# Install common dependencies manually
python3 -m pip install $PIP_FLAGS \
    pyyaml \
    jsonschema \
    requests \
    websocket-client \
    paho-mqtt \
    aiohttp
```

### 2.3 Install Demo Dependencies (for testing)
```bash
# Install demo packages for examples and testing
# (run "sudo apt update" first if you skipped it in 2.1)
sudo apt install -y \
    ros-$ROS_DISTRO-demo-nodes-cpp \
    ros-$ROS_DISTRO-demo-nodes-py \
    ros-$ROS_DISTRO-example-interfaces \
    ros-$ROS_DISTRO-rclcpp-components \
    ros-$ROS_DISTRO-composition-interfaces
```

## Step 3: Build Process

### 3.1 Clean Build (Recommended for First Build)
```bash
cd ~/muto_ws

# Clean any previous build artifacts
rm -rf build/ install/ log/

# Perform clean build
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install

# Source the built workspace
source install/setup.bash
```

### 3.2 Incremental Build (for Development)
```bash
cd ~/muto_ws

# Build only changed packages
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install --packages-up-to muto_agent muto_composer muto_core muto_msgs

# Build specific package only
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install --packages-select muto_agent

# Build with debug symbols
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### 3.3 Optimized Build (for Performance)
```bash
cd ~/muto_ws

# Parallel build with optimization
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

# Build with specific optimizations
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install \
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
find install/muto_agent/lib install/muto_composer/lib install/muto_core/lib -type f -executable | sort
# Expected files:
# install/muto_agent/lib/muto_agent/commands
# install/muto_agent/lib/muto_agent/mqtt
# install/muto_agent/lib/muto_agent/muto_agent
# install/muto_agent/lib/muto_agent/symphony_provider
# install/muto_composer/lib/muto_composer/compose_plugin
# install/muto_composer/lib/muto_composer/launch_plugin
# install/muto_composer/lib/muto_composer/muto_composer
# install/muto_composer/lib/muto_composer/provision_plugin
# install/muto_core/lib/muto_core/twin

# Check Python packages
# (with --symlink-install the pure-Python packages are linked to the source tree,
#  so only the generated muto_msgs package is found here)
find install/ -name "muto_*" -type d | grep -E "site-packages|dist-packages"
```

### 4.2 Test Basic Functionality
```bash
# Source workspace
source ~/muto_ws/install/setup.bash

# Test package discovery
ros2 pkg list | grep muto
# Expected output (eclipse_muto is the meta package, muto_listener/muto_talker are the talker-listener sample stack):
# muto_agent
# muto_composer
# muto_core
# eclipse_muto
# muto_listener
# muto_msgs
# muto_paho_mqtt_vendor
# muto_talker

# Test executable discovery
ros2 pkg executables muto_agent
# muto_agent commands
# muto_agent mqtt
# muto_agent muto_agent
# muto_agent symphony_provider
ros2 pkg executables muto_composer
# muto_composer compose_plugin
# muto_composer launch_plugin
# muto_composer muto_composer
# muto_composer provision_plugin
ros2 pkg executables muto_core
# muto_core twin

# Test message types
ros2 interface list | grep muto_msgs
```

### 4.3 Run Basic Components
```bash
# Test each component individually (in separate terminals)

# Terminal 1: Test core (the muto_core package provides the `twin` executable)
ros2 run muto_core twin --ros-args --log-level debug

# Terminal 2: Test agent
ros2 run muto_agent muto_agent --ros-args --log-level debug

# Terminal 3: Test composer
ros2 run muto_composer muto_composer --ros-args --log-level debug

# Terminal 4: Check nodes are running
ros2 node list
# Expected output:
# /core_twin
# /muto_agent
# /muto_composer
```

## Step 5: Build Customization

### 5.1 Development Build Configuration
```bash
# Create development build script
cat > ~/muto_ws/build_dev.sh << 'EOF'
#!/bin/bash
set -e

echo "Building Eclipse Muto for development..."

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
cd ~/muto_ws

# Clean build artifacts
rm -rf build/ install/ log/

# Build with debug symbols and development flags
colcon build \
    --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor \
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

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
cd ~/muto_ws

# Clean build artifacts
rm -rf build/ install/ log/

# Build with optimizations
colcon build \
    --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor \
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

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
cd ~/muto_ws

# Clean build artifacts
rm -rf build/ install/ log/

# Build for testing
colcon build \
    --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor \
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
    --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor \
    --python-testing pytest \
    --event-handlers console_direct+ \
    --return-code-on-test-failure

# Show test results
colcon test-result --verbose

echo "Build and test complete!"
EOF

chmod +x ~/muto_ws/build_test.sh
```

## Step 6: Advanced Build Options

### 6.1 Static Analysis
The Muto packages are Python. Each one (`agent`, `composer`, `core`) ships its own `ruff.toml` and `mypy.ini`, so static analysis is done with ruff (linting) and mypy (type checking):

```bash
# Install the Python linters (pip options as in Step 2.2)
if [ "$ROS_DISTRO" = "humble" ]; then
    PIP_FLAGS="--user"
else
    PIP_FLAGS="--user --break-system-packages"
fi
python3 -m pip install $PIP_FLAGS ruff mypy
export PATH="$HOME/.local/bin:$PATH"   # where pip --user installs ruff/mypy

# Run both tools in each package, using the package's own configuration
for pkg in agent composer core; do
    echo "=== muto_$pkg"
    cd ~/muto_ws/src/muto/src/$pkg
    ruff check .
    mypy muto_$pkg
done
```

Both tools print the issues they find and a summary line per package (for example `Found 12 errors.` or `Success: no issues found`).

### 6.2 Sanitizer Builds
```bash
# Build with AddressSanitizer
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer" \
    -DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer"

# Build with ThreadSanitizer
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install \
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
rosdep install --from-paths src src/muto/src/agent/muto_paho_mqtt_vendor --ignore-src -r -y

# For specific packages
sudo apt search ros-$ROS_DISTRO-package-name
sudo apt install ros-$ROS_DISTRO-package-name
```

#### Error: Python import failures
```bash
# Symptom: "ModuleNotFoundError: No module named 'xyz'"
# Solution: Install Python dependencies
# (on Jazzy / Ubuntu 24.04 add --break-system-packages, or install inside a virtual environment)
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

colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

### 7.2 Build Performance Issues

#### Slow Build Times
```bash
# Use parallel builds
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --parallel-workers $(nproc)

# Use ccache for faster rebuilds
sudo apt install -y ccache
export PATH="/usr/lib/ccache:$PATH"

# Build only changed packages
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --packages-up-to changed_package
```

#### Memory Issues During Build
```bash
# Limit parallel workers to reduce memory usage
colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --parallel-workers 2

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
rosdep check --from-paths src src/muto/src/agent/muto_paho_mqtt_vendor --ignore-src

# Install missing dependencies
rosdep install --from-paths src src/muto/src/agent/muto_paho_mqtt_vendor --ignore-src -r -y --skip-keys="package_to_skip"
```

#### Python Environment Conflicts
```bash
# Check Python version
python3 --version

# Check pip version
pip3 --version

# Reset pip if corrupted (Humble; on Jazzy use a virtual environment instead, see below)
python3 -m pip install --user --upgrade pip

# Use virtual environment if needed (recommended on Jazzy / Ubuntu 24.04, where pip refuses to modify the system Python)
# (--system-site-packages keeps the ROS 2 Python packages such as rclpy visible)
# The venv module is not installed by default on Ubuntu; install it first
sudo apt install -y python3-venv
python3 -m venv --system-site-packages ~/muto_venv
source ~/muto_venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install ~/muto_ws/src/muto/src/agent/symphony-sdk-python
python3 -m pip install pyyaml jsonschema requests websocket-client paho-mqtt aiohttp
```

## Step 8: Build Automation

### 8.1 Continuous Integration Build Script
```bash
# Create CI build script
cat > ~/muto_ws/ci_build.sh << 'EOF'
#!/bin/bash
set -euo pipefail

echo "=== Eclipse Muto CI Build ==="

# Environment setup (ROS setup scripts reference unset variables, so relax `set -u` while sourcing)
set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
set -u
cd ~/muto_ws

# Clean workspace
rm -rf build/ install/ log/

# Install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src src/muto/src/agent/muto_paho_mqtt_vendor --ignore-src -r -y

# Build
colcon build \
    --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_cohesion+ \
    --parallel-workers $(nproc)

# Test
set +u
source install/setup.bash
set -u
colcon test --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --python-testing pytest --event-handlers console_direct+ --return-code-on-test-failure

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
        source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
        cd ~/muto_ws
        rosdep install --from-paths src src/muto/src/agent/muto_paho_mqtt_vendor --ignore-src -r -y
        ;;
    "build")
        echo "Building for development..."
        source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
        cd ~/muto_ws
        colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
        source install/setup.bash
        ;;
    "test")
        echo "Running tests..."
        source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
        cd ~/muto_ws
        source install/setup.bash
        colcon test --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --python-testing pytest --event-handlers console_direct+
        ;;
    "clean")
        echo "Cleaning workspace..."
        cd ~/muto_ws
        rm -rf build/ install/ log/
        ;;
    "format")
        echo "Formatting code..."
        export PATH="$HOME/.local/bin:$PATH"   # where pip --user installs black/isort
        if [ "${ROS_DISTRO:-humble}" = "humble" ]; then
            python3 -m pip install --user black isort
        else
            python3 -m pip install --user --break-system-packages black isort
        fi
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

Start the whole Muto system and deploy a one-node test stack.

```bash
# Terminal 1: start Muto (agent, gateway, twin, composer and plugins in the /muto namespace)
source ~/muto_ws/install/setup.bash
ros2 launch ~/muto_ws/src/muto/launch/muto.launch.py vehicle_name:=test-robot enable_symphony:=false
```

```bash
# Terminal 2: check the Muto nodes, then deploy a stack with a single talker node
source ~/muto_ws/install/setup.bash
ros2 node list | grep muto

ros2 topic pub --once /muto/stack muto_msgs/msg/MutoAction \
  "{method: 'start', payload: '{\"metadata\": {\"name\": \"test\", \"content_type\": \"stack/json\"}, \"launch\": {\"node\": [{\"name\": \"test_talker\", \"pkg\": \"demo_nodes_cpp\", \"exec\": \"talker\"}]}}'}"

# After a few seconds the talker started by Muto should be listed
ros2 node list | grep test_talker
# Expected output: /test_talker
```

Stop Muto with Ctrl+C in Terminal 1.

## Conclusion

You now have Eclipse Muto built from source with multiple build configurations and verification scripts. Here's a summary of what you accomplished:

### ✅ Build Outputs
- **Executables**: `install/*/lib/*/*` - Ready to run Muto components
- **Python Packages**: `install/*/lib/python*/site-packages/` (Jazzy) or `install/*/local/lib/python*/dist-packages/` (Humble, generated `muto_msgs`) - Importable modules
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
# Verify integration: see Step 9.1
```

Your Eclipse Muto build is now ready for development, testing, and deployment!