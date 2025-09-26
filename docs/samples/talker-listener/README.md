
# Talker–Listener Sample

This is the classic minimal ROS 2 example with two nodes: a publisher (talker) and a subscriber (listener). The talker publishes incrementing messages; the listener prints them.
Run manually (outside Symphony/Muto) like this:

```bash
# Talker-Listener Sample

The classic ROS 2 communication example demonstrating publisher/subscriber pattern using Eclipse Muto orchestration and Eclipse Symphony fleet management.

## Overview

This sample consists of two nodes:
- **Talker**: Publishes incrementing "Hello world" messages on the `/chatter` topic
- **Listener**: Subscribes to messages and prints them to console

This demonstrates the fundamental ROS 2 communication pattern while showcasing how Muto can orchestrate node lifecycle and Symphony can manage deployment across robot fleets.

## Prerequisites

- Eclipse Muto workspace set up (see [Quick Start Guide](../../muto-quickstart.md))
- Eclipse Symphony running locally or remotely
- `demo_nodes_cpp` package available (included in standard ROS 2 installations)

## Manual Execution (Native ROS 2)

To understand the basic functionality before orchestration:

```bash
# Terminal 1: Start the listener
ros2 run demo_nodes_cpp listener

# Terminal 2: Start the talker  
ros2 run demo_nodes_cpp talker
```

Expected output:
```
[INFO] [talker]: Publishing: "Hello world: 1"
[INFO] [listener]: I heard: [Hello world: 1] 
[INFO] [talker]: Publishing: "Hello world: 2"
[INFO] [listener]: I heard: [Hello world: 2] 
```

## Deployment with Muto & Symphony

### Stack Definition

The Muto stack definition declares both nodes in JSON format. See the complete file at [`talker-listener.json`](../../symphony/talker-listener.json).

### Deployment Steps

1. **Follow the main quick start** to get Muto + Symphony running:
   ```bash
   # See: ../../muto-quickstart.md for complete setup
   ```

2. **Deploy the solution** using the helper scripts:
   ```bash
   cd ../../symphony
   ./define-solution.sh talker-listener.json
   ./define-instance.sh
   ```

3. **Monitor deployment** via Symphony API:
   ```bash
   curl http://localhost:8082/v1alpha2/instances
   curl http://localhost:8082/v1alpha2/targets
   ```

4. **Observe ROS topics** once deployed:
   ```bash
   ros2 topic list
   ros2 topic echo /chatter
   ```

### What Happens

1. **Target Registration**: Muto agent automatically registers the robot as a Symphony target
2. **Solution Creation**: The script packages the stack definition into a Symphony solution  
3. **Instance Deployment**: Symphony binds the solution to the target, triggering Muto to orchestrate the nodes
4. **Lifecycle Management**: Muto composer manages node startup, monitoring, and cleanup

## Troubleshooting

- **Nodes don't start**: Check that `demo_nodes_cpp` is installed and available in the container/environment
- **No topic communication**: Verify ROS domain ID consistency and network configuration
- **Symphony deployment fails**: Confirm Symphony services are running and accessible
- **Stack doesn't load**: Validate JSON syntax and ensure package names match installed packages

## Next Steps

- Try the [AprilTag Detection Sample](../april-tag-robot/README.md) for a more complex computer vision example
- Explore custom stack definitions for your own ROS packages
- Learn about Symphony's canary deployment and rollback capabilities

## References

- [ROS 2 demo_nodes_cpp Documentation](https://docs.ros.org/en/humble/p/demo_nodes_cpp/)
- [Eclipse Muto Architecture](../../../README.md#core-architecture) 
- [Symphony Integration Guide](../../muto-quickstart.md) 
````

## Running with Muto & Symphony

This is the default demo in the main quick start guide. Artifacts:
- Stack & solution definition: [`../../samples/talker-listener`](../../samples/talker-listener/)
- Scripts: [`../../samples/define-solution.sh`](../../samples/define-solution.sh), [`../../samples/define-instance.sh`](../../samples/define-instance.sh)

See also the upstream ROS package docs: [demo_nodes_cpp](https://docs.ros.org/en/iron/p/demo_nodes_cpp/)