# Eclipse Muto - Core Reference

**Core** provides fundamental services and abstractions for device/twin management and orchestration within the Eclipse Muto ecosystem. It serves as the bridge between physical devices and their digital twin representations.

## Overview

The Core component acts as the central service layer for Eclipse Muto, responsible for:
- Digital twin service implementation and management
- Device-to-cloud state synchronization
- Twin data persistence and retrieval
- Service interface abstraction for other Muto components
- Bootstrap and configuration management for Muto deployments

## Architecture

The Core architecture is designed for simplicity and reliability:

```
Core
├── Twin (twin.py)                 # Main twin service node
├── Twin Services (twin_services.py)  # Service implementation logic
├── Configuration (core.yaml)     # Core service configuration
└── Launch (core.launch.py)      # Launch file for Core services
```

## Component Reference

### Twin Service (`twin.py`)

**Purpose**: Main ROS 2 node providing digital twin services and device management capabilities.

**Key Responsibilities**:
- Expose digital twin service interface to other Muto components
- Manage device registration and identification
- Handle twin data queries and updates
- Provide bootstrap configuration for device initialization
- Integrate with cloud-based digital twin systems

**Service Interface**: `CoreTwin`
- **Request**: `input` (string) - Query or identifier string
- **Response**: `output` (string) - JSON response data

**Key Features**:
- **Device Registration**: Automatic device registration with digital twin
- **Configuration Retrieval**: Fetch device-specific configuration from twin
- **State Synchronization**: Sync device state with cloud twin
- **Bootstrap Support**: Provide initial stack configuration for device startup

**Node Details**:
```python
class Twin(Node):
    def __init__(self):
        super().__init__("core_twin")
        
        # Service for twin operations
        self.twin_service = self.create_service(
            CoreTwin, 
            "muto_twin", 
            self.handle_twin_request
        )
        
        # Initialize twin services
        self.twin_services = TwinServices()
```

**Configuration Parameters**:
- `twin_url`: Digital twin service endpoint (e.g., "http://ditto:8080")
- `namespace`: Device namespace identifier 
- `name`: Device name identifier
- `thing_id`: Digital twin thing identifier

**Service Operations**:
- **Stack Retrieval**: Get stack definitions by ID from digital twin
- **Device Info**: Retrieve device information and metadata
- **Configuration**: Get device-specific configuration parameters
- **State Updates**: Update device state in digital twin

### Twin Services (`twin_services.py`)

**Purpose**: Implementation layer providing the actual digital twin service logic and external system integration.

**Key Responsibilities**:
- HTTP client implementation for digital twin REST APIs
- Data transformation between ROS 2 messages and twin formats
- Error handling and retry logic for twin operations
- Authentication and authorization for twin services
- Response parsing and validation

**Key Methods**:
```python
class TwinServices:
    def get_stack_by_id(self, stack_id: str) -> str:
        """Retrieve stack definition from digital twin by ID."""
        
    def get_device_info(self, device_id: str) -> dict:
        """Get device information from digital twin."""
        
    def update_device_state(self, device_id: str, state: dict) -> bool:
        """Update device state in digital twin."""
        
    def register_device(self, device_info: dict) -> str:
        """Register device with digital twin system."""
```

**Integration Patterns**:
- **Eclipse Ditto**: Integration with Eclipse Ditto digital twin platform
- **HTTP REST**: Standard REST API communication patterns
- **JSON Processing**: Robust JSON parsing and validation
- **Error Handling**: Comprehensive error handling with retries

**Authentication Support**:
- Basic authentication with username/password
- Token-based authentication
- Anonymous access for development
- TLS/SSL certificate validation

## Digital Twin Integration

### Eclipse Ditto Integration

Core provides seamless integration with Eclipse Ditto digital twin platform:

**Thing Structure**:
```json
{
  "thingId": "org.eclipse.muto:device-001",
  "features": {
    "stack": {
      "properties": {
        "current": {
          "stackId": "example:stack:v1.0",
          "state": "deployed",
          "version": "1.0.0"
        }
      }
    },
    "device": {
      "properties": {
        "type": "robot",
        "location": "warehouse-a",
        "status": "online"
      }
    }
  }
}
```

**API Endpoints**:
- **Get Stack**: `GET /api/2/things/{thingId}/features/stack/properties/current`
- **Update State**: `PUT /api/2/things/{thingId}/features/device/properties`
- **Get Device**: `GET /api/2/things/{thingId}`

### Service Request Patterns

**Stack Retrieval Request**:
```python
# Request
request = CoreTwin.Request()
request.input = "example:stack:v1.0"

# Response
{
  "stackId": "example:stack:v1.0", 
  "name": "Example Stack",
  "nodes": [...],
  "metadata": {...}
}
```

**Device Information Request**:
```python
# Request  
request = CoreTwin.Request()
request.input = "device_info"

# Response
{
  "device_id": "device-001",
  "type": "robot",
  "location": "warehouse-a", 
  "capabilities": ["navigation", "manipulation"],
  "status": "online"
}
```

## Configuration

### Core Configuration (`config/core.yaml`)

```yaml
core:
  twin:
    # Digital twin service endpoint
    url: "http://ditto:8080"
    
    # Authentication settings
    auth:
      username: "ditto"
      password: "ditto"
      # or for token-based auth:
      # token: "your-auth-token"
    
    # Connection settings
    timeout: 30
    retry_attempts: 3
    retry_delay: 5
    
    # Device identification
    thing_id_template: "{namespace}:{name}"
    
    # API settings
    api_version: "2"
    content_type: "application/json"

  device:
    # Device metadata
    type: "robot"
    capabilities: ["navigation", "perception"]
    location: "production-floor"
    
    # Registration settings
    auto_register: true
    registration_retry: true
```


## Usage Patterns

### Service Client Usage

**Python Client**:
```python
import rclpy
from rclpy.node import Node
from muto_msgs.srv import CoreTwin

class TwinClient(Node):
    def __init__(self):
        super().__init__('twin_client')
        self.client = self.create_client(CoreTwin, 'muto_twin')
        
    async def get_stack(self, stack_id: str):
        request = CoreTwin.Request()
        request.input = stack_id
        
        future = self.client.call_async(request)
        response = await future
        
        return response.output

# Usage
client = TwinClient()
stack_data = await client.get_stack("example:stack:v1.0")
```

**Command Line Usage**:
```bash
# Get stack by ID
ros2 service call /muto/core_twin muto_msgs/srv/CoreTwin \
  "{input: 'example:stack:v1.0'}"

# Get device information
ros2 service call /muto/core_twin muto_msgs/srv/CoreTwin \
  "{input: 'device_info'}"

# Test service availability
ros2 service list | grep muto_twin
ros2 service type /muto/core_twin
```

### Integration with Other Components

**Composer Integration**:
```python
# Composer uses Core to retrieve stacks
class MutoComposer(Node):
    def __init__(self):
        self.get_stack_cli = self.create_client(CoreTwin, "core_twin")
    
    def bootstrap(self):
        """Bootstrap device with default stack from twin."""
        req = CoreTwin.Request()
        req.input = "default_stack"
        future = self.get_stack_cli.call_async(req)
        future.add_done_callback(self.activate)
```

**Agent Integration**:
```python
# Agent can query device configuration
class MutoAgent(Node):
    def __init__(self):
        self.twin_client = self.create_client(CoreTwin, "core_twin")
        
    async def get_device_config(self):
        request = CoreTwin.Request()
        request.input = "device_config"
        response = await self.twin_client.call_async(request)
        return json.loads(response.output)
```

## API Reference

### CoreTwin Service Interface

**Service Definition** (`muto_msgs/srv/CoreTwin.srv`):
```
string input
---
string output
```

**Request Types**:

1. **Stack Retrieval**:
   - Input: Stack ID (e.g., "example:stack:v1.0")
   - Output: JSON stack definition

2. **Device Information**:
   - Input: "device_info"
   - Output: JSON device metadata

3. **Configuration**:
   - Input: "config" or specific config key
   - Output: JSON configuration data

4. **Status Check**:
   - Input: "status"
   - Output: JSON status information

**Response Format**:
All responses are JSON strings with the following structure:
```json
{
  "success": true,
  "data": { /* response data */ },
  "error": null,
  "timestamp": "2025-01-01T12:00:00Z"
}
```

**Error Responses**:
```json
{
  "success": false,
  "data": null,
  "error": {
    "code": "NOT_FOUND",
    "message": "Stack not found",
    "details": "Stack ID 'invalid:stack' not found in digital twin"
  },
  "timestamp": "2025-01-01T12:00:00Z"
}
```


## Troubleshooting

### Common Issues

**1. Twin Service Connection Failures**:
```bash
# Check service status
ros2 service list | grep muto_twin

# Test connectivity to digital twin
curl -u ditto:ditto http://ditto:8080/api/2/things

# Check Core node logs
ros2 node info /muto/core_twin
```

**2. Authentication Issues**:
```yaml
# Verify credentials in config
core:
  twin:
    auth:
      username: "correct_username"
      password: "correct_password"
```

**3. JSON Parsing Errors**:
```bash
# Test service response manually
ros2 service call /muto/core_twin muto_msgs/srv/CoreTwin \
  "{input: 'test_request'}"

# Check response format
echo "Response should be valid JSON"
```

### Debug Commands

```bash
# Check Core service health
ros2 node info /muto/core_twin

# Monitor service calls
ros2 topic echo /rosout | grep core_twin

# Test service directly
ros2 service call /muto/core_twin muto_msgs/srv/CoreTwin \
  "{input: 'status'}"

# Check digital twin connectivity
curl -v http://ditto:8080/api/2/things/test:device
```

---

This comprehensive Core reference provides detailed information about Eclipse Muto's Core component, covering digital twin integration, service interfaces, configuration, and usage patterns for effective device-to-cloud connectivity.