# Eclipse Muto Agent - Reference Documentation

The **Muto Agent** is the intelligent communication bridge and orchestration coordinator that runs on each edge device. It serves as the primary gateway between cloud orchestration platforms and on-device ROS systems.

**Source Code**: [`src/agent/`](../../../../src/agent/)

## Architecture Overview

The Agent implements a sophisticated message routing architecture that provides:

- **Protocol Abstraction**: Support for multiple communication protocols (MQTT, HTTP, future Zenoh/uProtocol)
- **Message Routing**: Centralized message routing with specialized handlers
- **Security Management**: Authentication, encryption, and secure communication
- **Cloud Integration**: Native Eclipse Symphony orchestration support
- **State Synchronization**: Digital twin integration and state management

### Core Responsibilities

1. **Message Coordination**: Central message router for all system communication
2. **Protocol Gateway**: Protocol-agnostic interface to cloud systems
3. **Security Enforcement**: Authentication and authorization management
4. **Model Delivery**: Receive and process declarative stack models from cloud
5. **Status Reporting**: Report device and stack status to cloud platforms

## Component Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Muto Agent                               │
├─────────────────────────────────────────────────────────────┤
│  Message Handlers                                           │
│  ├─ GatewayMessageHandler    ─ Process cloud messages       │
│  ├─ ComposerMessageHandler   ─ Manage composer communication│
│  └─ CommandMessageHandler    ─ Handle command execution     │
├─────────────────────────────────────────────────────────────┤
│  Protocol Support                                           │
│  ├─ MQTT Manager            ─ MQTT communication            │
│  ├─ HTTP Client             ─ REST API communication        │
│  └─ Symphony Provider       ─ Eclipse Symphony integration  │
├─────────────────────────────────────────────────────────────┤
│  Core Services                                              │
│  ├─ Configuration Manager   ─ Parameter and config handling │
│  ├─ Topic Parser           ─ Message routing and parsing    │
│  └─ Command Executor       ─ Secure command execution       │
└─────────────────────────────────────────────────────────────┘
```

## Core Components

### MutoAgent (`muto_agent.py`)

The main Agent class that coordinates all system communication and message routing.

```python
class MutoAgent(BaseNode):
    """
    Main Muto Agent providing centralized message routing and coordination.
    
    The MutoAgent acts as a message router between different components
    including gateways, composers, and command processors.
    """
    
    def __init__(self, node_name: str = "muto_agent"):
        super().__init__(node_name)
        
        # Initialize message handlers
        self.gateway_handler = GatewayMessageHandler(self)
        self.composer_handler = ComposerMessageHandler(self) 
        self.command_handler = CommandMessageHandler(self)
        
        # Setup communication infrastructure
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_services()
```

**Key Methods:**

- `route_message()`: Central message routing logic
- `handle_gateway_message()`: Process messages from cloud gateways
- `handle_composer_response()`: Handle responses from composer
- `execute_command()`: Secure command execution interface

### Message Handlers

The Agent uses specialized message handlers for different communication patterns:

#### GatewayMessageHandler (`message_handlers.py`)

Processes messages from cloud orchestration platforms and gateways.

```python
class GatewayMessageHandler(MessageHandler):
    """Handles messages from cloud gateways and orchestration platforms"""
    
    def handle_message(self, message: Gateway) -> bool:
        """
        Process gateway messages including stack deployments and commands.
        
        Args:
            message: Gateway message from cloud platform
            
        Returns:
            True if message processed successfully
        """
        
    def validate_message(self, message: Gateway) -> bool:
        """Validate message format and authentication"""
        
    def route_to_composer(self, action: MutoAction) -> None:
        """Forward stack actions to composer"""
```

#### ComposerMessageHandler (`message_handlers.py`)

Manages communication with the Muto Composer for stack orchestration.

```python
class ComposerMessageHandler(MessageHandler):
    """Manages communication with Muto Composer"""
    
    def handle_composer_response(self, response: String) -> None:
        """Process responses from composer operations"""
        
    def send_stack_action(self, action: MutoAction) -> bool:
        """Send stack deployment action to composer"""
        
    def monitor_stack_status(self) -> dict:
        """Monitor current stack deployment status"""
```

#### CommandMessageHandler (`message_handlers.py`)

Handles secure command execution and routing.

```python
class CommandMessageHandler(MessageHandler):
    """Handles secure command execution and routing"""
    
    def execute_command(self, command: str, args: list) -> Tuple[int, str]:
        """Execute command with security validation"""
        
    def validate_command(self, command: str) -> bool:
        """Validate command against security policies"""
```

### MQTT Manager (`mqtt_manager.py`)

Manages MQTT communication infrastructure with comprehensive features:

```python
class MQTTManager:
    """
    Comprehensive MQTT communication manager with security and reliability.
    """
    
    def __init__(self, config: AgentConfig):
        self.client = mqtt.Client()
        self.config = config
        self._setup_security()
        self._setup_callbacks()
    
    def connect(self) -> bool:
        """Establish secure MQTT connection"""
        
    def publish(self, topic: str, payload: str, qos: int = 1) -> bool:
        """Publish message with quality of service guarantees"""
        
    def subscribe(self, topic: str, callback: callable) -> bool:
        """Subscribe to topic with callback handling"""
```

**Features:**
- **TLS Encryption**: Secure communication with certificate validation
- **Quality of Service**: Configurable message delivery guarantees
- **Connection Management**: Automatic reconnection and error recovery
- **Topic Management**: Dynamic topic subscription and unsubscription

### Configuration Manager (`config.py`)

Centralized configuration management with ROS parameter integration:

```python
class ConfigurationManager:
    """Manages Agent configuration with ROS parameter integration"""
    
    def __init__(self, node):
        self.node = node
        self.config = AgentConfig()
        self._load_parameters()
    
    def get_config(self) -> AgentConfig:
        """Get current agent configuration"""
        
    def update_parameter(self, name: str, value: any) -> bool:
        """Update configuration parameter dynamically"""
        
    def validate_config(self) -> bool:
        """Validate configuration completeness and correctness"""
```

### Topic Parser (`topic_parser.py`)

Advanced message routing and topic parsing capabilities:

```python
class MutoTopicParser:
    """Advanced topic parsing and message routing"""
    
    def parse_topic(self, topic: str) -> dict:
        """Parse topic structure and extract routing information"""
        
    def route_message(self, topic: str, message: any) -> bool:
        """Route message based on topic structure"""
        
    def validate_topic_format(self, topic: str) -> bool:
        """Validate topic format against Muto standards"""
```

## Plugin Architecture

### Protocol Plugins

The Agent supports extensible protocol plugins for different communication methods:

```python
class ProtocolPlugin:
    """Base class for protocol plugins"""
    
    def connect(self) -> bool:
        """Establish connection using this protocol"""
        
    def send_message(self, message: any) -> bool:
        """Send message using this protocol"""
        
    def handle_received_message(self, message: any) -> None:
        """Handle received message from this protocol"""
```

#### MQTT Plugin (`mqtt.py`)
- **Broker Support**: Mosquitto, HiveMQ, AWS IoT Core
- **Security**: TLS 1.2+, certificate-based authentication
- **Reliability**: Persistent sessions, message queuing
- **Performance**: Optimized for fleet-scale communication

#### HTTP Plugin (planned)
- **REST API**: RESTful communication patterns
- **Authentication**: JWT tokens, API keys
- **Load Balancing**: Multi-endpoint support
- **Caching**: Request/response caching for performance

#### uProtocol/uServices Plugin (planned)
- Under investigation as and laternative to using mqtt and symphony provider over mqtt

### Cloud Orchestration Providers

#### Symphony Provider (`symphony/symphony_provider.py`)

Complete Eclipse Symphony integration with enterprise features:

```python
class MutoSymphonyProvider(BaseNode):
    """
    Eclipse Symphony integration provider with comprehensive
    COA (Component Operational Agreement) protocol support.
    """
    
    def __init__(self):
        super().__init__('muto_symphony_provider')
        self.sdk = SymphonySDK()
        self.api = SymphonyAPI()
        
    def handle_coa_request(self, request: dict) -> dict:
        """Handle Component Operational Agreement requests"""
        
    def register_target(self, target_info: dict) -> bool:
        """Register device as Symphony target"""
        
    def report_status(self, status: dict) -> bool:
        """Report device status to Symphony"""
```

**Symphony SDK Components:**

#### API Client (`symphony/sdk/symphony_api.py`)
```python
class SymphonyAPI:
    """REST API client for Eclipse Symphony"""
    
    def authenticate(self, username: str, password: str) -> str:
        """Authenticate and get access token"""
        
    def create_target(self, target_def: dict) -> dict:
        """Create new target in Symphony"""
        
    def deploy_solution(self, instance_def: dict) -> dict:
        """Deploy solution instance to target"""
        
    def get_deployment_status(self, instance_id: str) -> dict:
        """Get deployment status and results"""
```

#### Data Structures (`symphony/sdk/symphony_sdk.py`)
```python
@dataclass
class TargetSpec:
@dataclass  
class SolutionSpec:
@dataclass
class ComponentSpec:
```

## Configuration Reference

### Core Parameters

```yaml
/**:
  ros__parameters:
    # Core topics for message routing
    stack_topic: "stack"                    # Stack deployment topic
    twin_topic: "twin"                      # Digital twin topic
    agent_to_gateway_topic: "agent_to_gateway"  # Agent→Gateway
    gateway_to_agent_topic: "gateway_to_agent"  # Gateway→Agent
    
    # Device identity configuration
    namespace: "org.eclipse.muto"           # Device namespace
    name: "muto-device-001"                 # Unique device identifier
    type: "mobile_robot"                    # Device type classification
    attributes: '{"brand": "acme", "model": "robot-x"}'  # JSON attributes
    
    # MQTT configuration
    host: "mqtt.example.com"                # MQTT broker hostname
    port: 1883                              # MQTT broker port (1883/8883)
    username: "device001"                   # MQTT username
    password: "secure_password"             # MQTT password
    use_tls: true                          # Enable TLS encryption
    ca_cert_path: "/certs/ca.pem"          # CA certificate path
    
    # Symphony integration
    symphony_enabled: true                  # Enable Symphony integration
    symphony_auto_register: false          # Auto-register as Symphony target
    symphony_api_url: "https://symphony.example.com/api/v1alpha2/"
    symphony_provider_name: "providers.target.mqtt"
    symphony_target_name: "muto-device-001"
    symphony_timeout_seconds: 30
    
    # Performance tuning
    message_queue_size: 100                 # Message queue buffer size
    connection_retry_attempts: 5            # Connection retry count
    heartbeat_interval: 30                  # Heartbeat interval (seconds)
```

### Advanced Configuration

#### Security Settings
```yaml
/**:
  ros__parameters:
    # TLS/SSL configuration
    tls_version: "TLSv1.2"                 # Minimum TLS version
    verify_hostname: true                   # Verify server hostname
    cert_reqs: "CERT_REQUIRED"             # Certificate requirements
    
    # Authentication
    auth_method: "certificate"              # certificate|password|token
    client_cert_path: "/certs/client.pem"
    client_key_path: "/certs/client.key"
    
    # Command execution security
    allowed_commands: ["ros2", "colcon"]    # Whitelist of allowed commands
    command_timeout: 60                     # Command execution timeout
    sandbox_execution: true                 # Enable command sandboxing
```

#### Performance Optimization
```yaml
/**:
  ros__parameters:
    # Message handling
    worker_thread_count: 4                  # Number of worker threads
    message_batch_size: 10                  # Batch processing size
    max_message_size: 1048576              # Max message size (1MB)
    
    # Connection management
    keep_alive_interval: 60                 # MQTT keep-alive interval
    connection_timeout: 30                  # Connection timeout
    reconnect_delay: 5                     # Reconnection delay
    
    # Memory management
    message_history_size: 1000             # Message history buffer
    log_rotation_size: "10MB"              # Log file rotation size
```

## API Reference

### ROS Topics

#### Publications
```python
# Stack actions to composer
/muto/stack (muto_msgs/MutoAction)

# Digital twin updates
/muto/twin (muto_msgs/Thing)

# Gateway communication
/muto/agent_to_gateway (muto_msgs/Gateway)
```

#### Subscriptions
```python
# Gateway messages from cloud
/muto/gateway_to_agent (muto_msgs/Gateway)

# Composer responses and status
/muto/composer_response (std_msgs/String)

# Twin synchronization messages
/muto/twin_updates (muto_msgs/Thing)
```

### ROS Services

#### Agent Services
```python
# Configuration management
/muto/agent/get_config (muto_msgs/srv/GetConfig)
/muto/agent/set_config (muto_msgs/srv/SetConfig)

# Status and health
/muto/agent/get_status (muto_msgs/srv/GetStatus)
/muto/agent/health_check (muto_msgs/srv/HealthCheck)

# Command execution
/muto/agent/execute_command (muto_msgs/srv/ExecuteCommand)
```

### Message Formats

#### MutoAction Message
```python
# Stack deployment action
std_msgs/Header header
string action                # "start"|"stop"|"reload"|"status"
string stack                 # JSON stack definition or stack ID
string context              # Deployment context information
```

#### Gateway Message
```python
# Cloud gateway communication
std_msgs/Header header
string source               # Message source identifier
string destination          # Message destination
string message_type         # Type of gateway message
string payload              # Message payload (JSON)
```

#### Thing Message
```python
# Digital twin representation
std_msgs/Header header
string namespace            # Device namespace
string name                 # Device name
string type                 # Device type
string state                # Current device state
string properties          # JSON properties
```

## Error Handling

### Exception Types

```python
class AgentError(Exception):
    """Base exception for Agent errors"""
    
class ConfigurationError(AgentError):
    """Configuration-related errors"""
    
class CommunicationError(AgentError):
    """Communication protocol errors"""
    
class AuthenticationError(AgentError):
    """Authentication and security errors"""
    
class TopicParsingError(AgentError):
    """Topic parsing and routing errors"""
```

### Error Recovery Patterns

#### Connection Recovery
```python
def handle_connection_loss(self):
    """Handle MQTT connection loss with exponential backoff"""
    retry_count = 0
    while retry_count < self.max_retries:
        delay = min(2 ** retry_count, 60)  # Exponential backoff, max 60s
        time.sleep(delay)
        
        if self.reconnect():
            self.logger.info("Successfully reconnected")
            return True
            
        retry_count += 1
    
    self.logger.error("Failed to reconnect after max retries")
    return False
```

#### Message Processing Recovery
```python
def process_message_safe(self, message):
    """Process message with error recovery"""
    try:
        return self.process_message(message)
    except MessageProcessingError as e:
        self.logger.warning(f"Message processing failed: {e}")
        self.dead_letter_queue.append(message)
        return False
    except Exception as e:
        self.logger.error(f"Unexpected error: {e}")
        self.restart_message_handler()
        return False
```

## Security Model
See teh underlying protocol and orchestrator implementations:
[Eclipse Ditto]()
[Eclipse Symphony]()

### Authentication Methods

#### Certificate-Based Authentication
```yaml
/**:
  ros__parameters:
    auth_method: "certificate"
    client_cert_path: "/certs/device.pem"
    client_key_path: "/certs/device.key" 
    ca_cert_path: "/certs/ca.pem"
    verify_hostname: true
```

#### Token-Based Authentication
```yaml
/**:
  ros__parameters:
    auth_method: "token"
    auth_token: "eyJhbGciOiJIUzI1NiIs..."
    token_refresh_url: "https://auth.example.com/refresh"
```

### Authorization Model

#### Topic-Level Permissions
```python
topic_permissions = {
    "/muto/stack": ["publish", "subscribe"],
    "/muto/twin": ["publish", "subscribe"], 
    "/muto/gateway_to_agent": ["subscribe"],
    "/muto/agent_to_gateway": ["publish"]
}
```

#### Command Execution Security
```python
# Whitelist approach for command execution
allowed_commands = [
    "ros2 node list",
    "ros2 topic list", 
    "ros2 service list",
    "colcon build",
    "systemctl status"
]

# Sandbox execution environment
sandbox_config = {
    "user": "muto-agent",
    "working_directory": "/tmp/muto",
    "environment_whitelist": ["ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY"],
    "network_access": False,
    "filesystem_readonly": True
}
```



## Development and Testing

see the [Developer Guide](../../developer_guide/readme.md)


---

This reference documentation provides comprehensive technical details for the Eclipse Muto Agent component. For practical usage examples, see the [User Guide](../../user_guide/readme.md), and for development information, see the  [Developer Guide](../../developer_guide/readme.md).