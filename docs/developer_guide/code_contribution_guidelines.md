# Eclipse Muto - Code Contribution Guidelines

This guide provides comprehensive instructions for contributing code to the Eclipse Muto project, covering everything from initial setup to pull request submission.

## Overview

Eclipse Muto is an open-source project developed under the Eclipse Foundation. We welcome contributions from the community and are committed to fostering an inclusive and collaborative environment.

### Contribution Philosophy
- **Quality over Quantity**: Well-tested, documented contributions are preferred
- **Community First**: All contributors should feel welcome and supported
- **Transparency**: Development discussions happen in public forums
- **Standards Compliance**: Follow established coding and documentation standards

## Prerequisites

Before contributing, ensure you have:
- ✅ Completed [Development Environment Setup](./development_environment_setup.md)
- ✅ Successfully built and run Muto from source
- ✅ Familiarity with Git, GitHub, and ROS 2 development
- ✅ Signed the Eclipse Contributor Agreement (ECA)

## Step 1: Legal Requirements

### 1.1 Eclipse Contributor Agreement (ECA)
All contributors must sign the Eclipse Contributor Agreement:

1. Visit [Eclipse ECA](https://www.eclipse.org/legal/ECA.php)
2. Sign the agreement electronically
3. Verify your email address is associated with your Eclipse account
4. Use the same email for Git commits

### 1.2 Developer Certificate of Origin (DCO)
All commits must be signed off to certify you have the right to contribute:

```bash
# Configure Git with your ECA email
git config --global user.email "your-eca-email@example.com"
git config --global user.name "Your Name"

# Always use -s flag when committing
git commit -s -m "Your commit message"

# Or configure Git to always sign off
git config --global format.signoff true
```

### 1.3 Copyright Headers
All new files must include Eclipse copyright headers:

```python
# For Python files
# Copyright (c) 2024 Eclipse Muto Contributors
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License v. 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
```

```cpp
// For C++ files
// Copyright (c) 2024 Eclipse Muto Contributors
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License v. 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0
```

## Step 2: Development Workflow

### 2.1 Fork and Clone
```bash
# 1. Fork the repository on GitHub
# Visit https://github.com/eclipse-muto/muto and click "Fork"

# 2. Clone your fork
git clone https://github.com/YOUR_USERNAME/muto.git ~/muto_ws/src/muto
cd ~/muto_ws/src/muto

# 3. Add upstream remote
git remote add upstream https://github.com/eclipse-muto/muto.git

# 4. Verify remotes
git remote -v
# origin    https://github.com/YOUR_USERNAME/muto.git (fetch)
# origin    https://github.com/YOUR_USERNAME/muto.git (push)
# upstream  https://github.com/eclipse-muto/muto.git (fetch)
# upstream  https://github.com/eclipse-muto/muto.git (push)
```

### 2.2 Branch Strategy
```bash
# 1. Always work on feature branches
git checkout main
git pull upstream main

# 2. Create descriptive branch names
git checkout -b feature/add-mqtt-authentication
git checkout -b bugfix/fix-stack-validation-error
git checkout -b docs/update-installation-guide

# 3. Keep branches focused on single features/fixes
# Each branch should address one specific issue
```

### 2.3 Commit Guidelines
```bash
# Commit message format:
# <type>(<scope>): <description>
# 
# <body>
# 
# Signed-off-by: Your Name <your-eca-email@example.com>

# Examples of good commit messages:
git commit -s -m "feat(agent): add MQTT authentication support

Implements authentication mechanism for MQTT broker connections
including username/password and certificate-based authentication.

Fixes #123"

git commit -s -m "fix(composer): resolve stack validation timeout issue

Stack validation was timing out for complex stacks due to 
insufficient default timeout value. Increased default timeout
from 30s to 60s and made it configurable.

Fixes #456"

git commit -s -m "docs(readme): update installation instructions

Added missing dependency installation steps and clarified
container deployment prerequisites.

Closes #789"
```

## Step 3: Coding Standards

### 3.1 Python Code Standards
```python
# Use Black formatter with 88 character line limit
# Install: pip3 install --user black isort flake8 mypy

# Format code before committing
black src/
isort src/ --profile black

# Check linting
flake8 src/ --max-line-length=88 --extend-ignore=E203,W503

# Type checking (optional but recommended)
mypy src/
```

### 3.2 Python Style Guidelines
```python
"""Module docstring describing the module purpose.

This module implements the core functionality for Eclipse Muto
agent communication and stack management.
"""

import asyncio
import logging
from typing import Dict, List, Optional, Union

import rclpy
from rclpy.node import Node


class MutoAgent(Node):
    """Eclipse Muto Agent for device communication and orchestration.
    
    The MutoAgent class provides the main interface for device-level
    communication with cloud orchestration platforms like Eclipse Symphony.
    
    Attributes:
        config: Configuration dictionary loaded from YAML file
        symphony_provider: Optional Symphony integration provider
        mqtt_manager: MQTT communication manager
    """
    
    def __init__(self, config: Dict[str, any]) -> None:
        """Initialize the Muto Agent.
        
        Args:
            config: Configuration dictionary containing agent settings
            
        Raises:
            ValueError: If required configuration parameters are missing
            ConnectionError: If unable to connect to required services
        """
        super().__init__('muto_agent')
        self.config = config
        self._setup_logging()
        self._initialize_providers()
    
    def _setup_logging(self) -> None:
        """Configure logging based on configuration settings."""
        log_level = self.config.get('log_level', 'INFO')
        logging.basicConfig(
            level=getattr(logging, log_level.upper()),
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
    async def process_stack_request(
        self, 
        stack_definition: Dict[str, any]
    ) -> bool:
        """Process a stack deployment request.
        
        Args:
            stack_definition: JSON stack definition to deploy
            
        Returns:
            True if stack deployment initiated successfully, False otherwise
            
        Raises:
            ValidationError: If stack definition is invalid
        """
        try:
            # Implementation here
            pass
        except Exception as e:
            self.logger.error(f"Failed to process stack request: {e}")
            return False
```

### 3.3 C++ Code Standards (if applicable)
```cpp
// Use consistent indentation (2 spaces)
// Follow ROS 2 C++ style guide
// Use descriptive variable and function names

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "muto_msgs/msg/muto_action.hpp"

namespace muto {
namespace core {

class MutoCore : public rclcpp::Node {
public:
  explicit MutoCore(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MutoCore() = default;

private:
  void handle_stack_action(const muto_msgs::msg::MutoAction::SharedPtr msg);
  
  // Member variables use trailing underscore
  rclcpp::Subscription<muto_msgs::msg::MutoAction>::SharedPtr stack_subscription_;
  std::shared_ptr<MutoTwinService> twin_service_;
};

}  // namespace core
}  // namespace muto
```

## Step 4: Testing Requirements

### 4.1 Unit Testing
```python
# All new code must include unit tests
# Use pytest framework with at least 80% code coverage

# Example test file: test_muto_agent.py
import pytest
from unittest.mock import Mock, patch

from muto_agent.muto_agent import MutoAgent


class TestMutoAgent:
    """Test suite for MutoAgent class."""
    
    @pytest.fixture
    def mock_config(self):
        """Provide mock configuration for tests."""
        return {
            'agent': {
                'node_name': 'test_agent',
                'log_level': 'DEBUG'
            }
        }
    
    @pytest.fixture
    def muto_agent(self, mock_config):
        """Provide MutoAgent instance for tests."""
        with patch('rclpy.init'), patch('rclpy.create_node'):
            return MutoAgent(mock_config)
    
    def test_agent_initialization(self, muto_agent, mock_config):
        """Test agent initializes with correct configuration."""
        assert muto_agent.config == mock_config
        assert muto_agent.get_name() == 'test_agent'
    
    @pytest.mark.asyncio
    async def test_process_stack_request_valid(self, muto_agent):
        """Test processing valid stack request."""
        stack_def = {
            'name': 'Test Stack',
            'stackId': 'test:stack:v1.0',
            'node': [{'name': 'test_node', 'pkg': 'demo_nodes_cpp', 'exec': 'talker'}]
        }
        
        result = await muto_agent.process_stack_request(stack_def)
        assert result is True
    
    @pytest.mark.asyncio
    async def test_process_stack_request_invalid(self, muto_agent):
        """Test processing invalid stack request."""
        invalid_stack = {'invalid': 'definition'}
        
        result = await muto_agent.process_stack_request(invalid_stack)
        assert result is False
```

### 4.2 Integration Testing
```python
# Integration tests verify component interactions
# Use ROS 2 launch_testing framework

import unittest
import launch
import launch_testing
import rclpy

from muto_msgs.msg import MutoAction


class TestMutoIntegration(unittest.TestCase):
    """Integration tests for Muto components."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment."""
        rclpy.init()
        cls.node = rclpy.create_node('test_integration')
    
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment."""
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def test_stack_deployment_flow(self):
        """Test complete stack deployment flow."""
        # Create stack definition
        stack_msg = MutoAction()
        stack_msg.action = 'start'
        stack_msg.stack = '{"name":"Test","stackId":"test:v1"}'
        
        # Publish stack action
        publisher = self.node.create_publisher(MutoAction, '/muto/stack', 10)
        publisher.publish(stack_msg)
        
        # Wait for response
        # Implementation depends on specific test requirements
```

### 4.3 Running Tests
```bash
# Run all tests
cd ~/muto_ws
colcon test --event-handlers console_direct+

# Run specific package tests
colcon test --packages-select muto_agent --event-handlers console_direct+

# Run tests with coverage
colcon test --packages-select muto_agent --pytest-args --cov

# Check test results
colcon test-result --verbose
```

## Step 5: Documentation Requirements

Please follow the  [CODING_GUIDELINES.md](../../CODING_GUIDELINES.md)

## Step 6: Pull Request Process

### 6.1 Pre-submission Checklist
```bash
# Create pre-submission script
cat > ~/muto_ws/pre_submit_check.sh << 'EOF'
#!/bin/bash
set -e

echo "=== Eclipse Muto Pre-submission Checklist ==="

cd ~/muto_ws/src/muto

# 1. Check code formatting
echo "1. Checking code formatting..."
black --check src/ || (echo "Run 'black src/' to fix formatting" && exit 1)
isort --check-only src/ --profile black || (echo "Run 'isort src/ --profile black' to fix imports" && exit 1)

# 2. Check linting
echo "2. Running linting checks..."
flake8 src/ --max-line-length=88 --extend-ignore=E203,W503

# 3. Run tests
echo "3. Running tests..."
cd ~/muto_ws
colcon test --packages-select muto_agent muto_composer muto_core --event-handlers console_direct+
colcon test-result --verbose

# 4. Check commit signatures
echo "4. Checking commit signatures..."
git log --pretty=format:"%h %s" -n 5 | while read commit; do
    if ! git show --format="%B" $commit | grep -q "Signed-off-by:"; then
        echo "ERROR: Commit $commit is not signed off"
        exit 1
    fi
done

# 5. Check for copyright headers
echo "5. Checking copyright headers..."
find src/ -name "*.py" | while read file; do
    if ! head -5 "$file" | grep -q "Eclipse Public License"; then
        echo "WARNING: $file may be missing copyright header"
    fi
done

echo "✓ Pre-submission checks passed!"
EOF

chmod +x ~/muto_ws/pre_submit_check.sh
```

### 6.2 Creating Pull Request
```bash
# 1. Ensure branch is up to date
git checkout main
git pull upstream main
git checkout your-feature-branch
git rebase main

# 2. Run pre-submission checks
~/muto_ws/pre_submit_check.sh

# 3. Push to your fork
git push origin your-feature-branch

# 4. Create pull request on GitHub
# - Visit your fork on GitHub
# - Click "New Pull Request"
# - Select your branch
# - Fill out PR template
```

### 6.3 Pull Request Template
```markdown
## Description
Brief description of what this PR does.

## Related Issues
Fixes #123
Relates to #456

## Type of Change
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update

## Testing
- [ ] Unit tests pass
- [ ] Integration tests pass
- [ ] Manual testing performed

## Checklist
- [ ] Code follows project coding standards
- [ ] Self-review of the code completed
- [ ] Code is commented, particularly in hard-to-understand areas
- [ ] Corresponding changes to documentation made
- [ ] No new warnings introduced
- [ ] All commits are signed off
- [ ] Copyright headers included in new files

## Additional Notes
Any additional information, deployment notes, etc.
```

## Step 7: Review Process

### 7.1 Code Review Guidelines
- **Be Constructive**: Provide specific, actionable feedback
- **Be Respectful**: Maintain professional and respectful tone
- **Be Thorough**: Review both functionality and code quality
- **Be Timely**: Respond to review requests within 2-3 business days

### 7.2 Addressing Review Comments
```bash
# 1. Make requested changes
# Edit files based on review feedback

# 2. Test changes
~/muto_ws/pre_submit_check.sh

# 3. Commit changes (maintaining sign-off)
git add .
git commit -s -m "fix(agent): address review comments

- Improved error handling in stack validation
- Added missing type hints
- Updated documentation as requested"

# 4. Push updates
git push origin your-feature-branch

# PR will automatically update with new commits
```

### 7.3 Merge Requirements
Before a PR can be merged, it must:
- ✅ Pass all automated CI checks
- ✅ Have at least one approving review from a maintainer
- ✅ Have all review comments resolved
- ✅ Be up to date with main branch
- ✅ Include appropriate tests and documentation
- ✅ Follow all coding standards

## Step 8: Continuous Improvement

### 8.1 Learning from Reviews
- Study feedback patterns in your PRs
- Review other contributors' PRs to learn best practices
- Participate in design discussions and RFCs
- Ask questions when unsure about requirements

### 8.2 Becoming a Regular Contributor
- Start with small, well-defined issues
- Build trust through consistent quality contributions  
- Engage with the community through discussions
- Help review other contributors' PRs
- Consider becoming a maintainer after demonstrating expertise

## Common Pitfalls to Avoid

### ❌ Don't:
- Submit large PRs that change multiple unrelated things
- Ignore coding standards or skip tests
- Forget to sign off commits or include copyright headers
- Make breaking changes without discussion
- Submit PRs without testing on your local system

### ✅ Do:
- Keep PRs focused on single features or fixes
- Write comprehensive tests for new functionality
- Document your changes clearly
- Respond promptly to review feedback
- Follow up on your PRs until they're merged

## Getting Help

### Resources
- **GitHub Discussions**: [https://github.com/eclipse-muto/muto/discussions](https://github.com/eclipse-muto/muto/discussions)
- **Issue Tracker**: [https://github.com/eclipse-muto/muto/issues](https://github.com/eclipse-muto/muto/issues)
- **Eclipse Muto Project**: [https://projects.eclipse.org/projects/automotive.muto](https://projects.eclipse.org/projects/automotive.muto)
- **Developer Documentation**: [Developer Guide](./readme.md)

### Support Channels
- Create GitHub issues for bugs and feature requests
- Use GitHub discussions for questions and ideas
- Join Eclipse IoT working group meetings (when available)
- Reach out to maintainers for guidance on large contributions

## Recognition

Contributors to Eclipse Muto are recognized in:
- Git commit history
- Release notes for significant contributions
- Eclipse Foundation contributor profiles
- Project documentation and acknowledgments

Thank you for contributing to Eclipse Muto! Your contributions help advance the future of robot orchestration and edge computing.