# AprilTag Detection Sample

<img src="https://github.com/ros-misc-utilities/apriltag_detector/raw/master/images/apriltags.png" width="40%" alt="AprilTags example" />

Real-time AprilTag detection and visualization pipeline demonstrating computer vision workflows with Eclipse Muto orchestration and Eclipse Symphony fleet management.

## Overview

This sample implements a complete AprilTag detection system using:
- **Camera input**: Live camera feed or recorded bag files
- **Detection**: Fast AprilTag family detection (tag36h11, tagStandard41h12, etc.)
- **Visualization**: Optional detection overlay and tracking display
- **Orchestration**: Muto-managed composable node containers
- **Fleet Management**: Symphony-based remote deployment and updates

### Key Features

- **Lightweight perception**: Focus on detection without full 6-DoF pose estimation
- **Composable architecture**: Efficient memory sharing between vision nodes
- **Multi-family support**: Configurable tag families and detection parameters  
- **Remote deployment**: Over-the-air updates via Symphony orchestration
- **Telemetry ready**: Detection metrics and performance monitoring

> **Note**: This sample uses [apriltag_detector](https://github.com/ros-misc-utilities/apriltag_detector) for pure detection. For combined detection + pose estimation, consider [apriltag_ros](https://github.com/christianrauch/apriltag_ros).

## Artifact Map

Directory: [`samples/april-tag-robot/`](./)

| File | Purpose |
|------|---------|
| `apriltag-detector-muto-stack.json` | Declarative Muto stack (JSON) for detector + draw nodes (composable container) |
| `apriltag-tracking-solution.json` | Symphony Solution (wraps stack as base64 payload) |
| `apriltag-tracking-instance.json` | Symphony Instance binding Solution → Target |
| `apriltag_workspace/stack.json` | Alternative workspace/stack form (if using archive packaging) |
| `apriltag_workspace/launch/apriltag.launch.yaml` | Upstream style launch YAML (reference only) |
| `apriltag_workspace/config/tags_36h11.yaml` | Tag family parameter file |

Target definition is auto‑registered by the running Muto Agent (see talker–listener sample). For a static reference, review [`../talker-listener/test-robot-debug-target.json`](../talker-listener/test-robot-debug-target.json).

## Scenario Overview

You operate a fleet of robots each with a monocular camera. You need to:
1. Deploy an initial AprilTag detection capability (v1).
2. Safely roll out an improved pipeline (v2) that supports additional tag families and optional tracking/visualization.
3. Use canary rollout + rollback policies to limit risk.

The complete deep‑dive OTA narrative (architecture + sequence diagrams) lives in `ros_variant/muto.md` (search for "AprilTag" section). This README keeps a concise, actionable version focused on running the sample.

### Version Delta (Conceptual)
| Aspect | v1 (baseline) | v2 (enhanced) |
|--------|---------------|---------------|
| Families | `tag36h11` | `tag36h11`, `tagStandard41h12` |
| Nodes | detector, draw | detector, draw, (optional future tracker) |
| Telemetry | raw detections | detections + extended metrics |
| Policy | none | optional: only when docked |

Refer to the architecture + sequence diagrams in `muto.md` for full context. Keeping this README lean.

## Prerequisites

- **Camera hardware**: USB camera, Intel RealSense, or other ROS 2 compatible camera
- **AprilTag packages**: Install detection dependencies
  ```bash
  sudo apt install ros-$ROS_DISTRO-apriltag ros-$ROS_DISTRO-apriltag-msgs
  # Or build from source: https://github.com/ros-misc-utilities/apriltag_detector
  ```
- **Eclipse Muto + Symphony**: Complete setup from [Quick Start Guide](../../muto-quickstart.md)

## Manual Testing (Native ROS 2)

Before orchestrated deployment, verify the detection pipeline:

```bash
# Terminal 1: Start camera (adjust topic as needed)
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Start AprilTag detector  
ros2 run apriltag_detector apriltag_detector_node --ros-args \
  -r /image:=/image_raw \
  -p tag_family:=tag36h11

# Terminal 3: View detections
ros2 topic echo /apriltag/detections
```

## Deploy the Sample

### Step 1: Start Symphony + Muto
Follow the [Quick Start Guide](../../muto-quickstart.md) to get Muto + Symphony running. Ensure a target like `test-robot-debug` appears in Symphony:

```bash
# Verify Symphony API is running
curl http://localhost:8082/v1alpha2/greetings

# Check if Muto agent registered the target
curl -s http://localhost:8082/v1alpha2/targets | jq '.items[].metadata.name'
```

### Step 2: Review Stack Configuration
The AprilTag stack definition [`apriltag-detector-muto-stack.json`](./apriltag-detector-muto-stack.json) defines a composable container with two nodes:

- **apriltag_detector_node**: Detects AprilTags in camera images
- **apriltag_draw_node**: Draws detection overlays on images

Key configuration areas:
```json
{
  "composable": [
    {
      "container_name": "apriltag_container",
      "node": [
        {
          "package": "apriltag_detector", 
          "executable": "apriltag_detector_node",
          "parameters": [
            {"tag_family": "tag36h11"},
            {"tag_edge_size": 0.05}
          ],
          "remappings": [
            {"from": "/image", "to": "/camera/image_raw"}
          ]
        }
      ]
    }
  ]
}
```

Edit parameters as needed:
- `tag_family`: Supported families include `tag36h11`, `tagStandard41h12`
- `tag_edge_size`: Physical tag size in meters
- Image topic remapping to match your camera driver

### Step 3: Deploy Solution
Navigate to the samples directory and create the Solution:

```bash
cd samples/
./define-solution.sh april-tag-robot/apriltag-tracking-solution.json
```

The Solution wraps the stack JSON as a base64-encoded payload. If you modified the stack definition, regenerate the encoding:

```bash
cd april-tag-robot/
STACK=apriltag-detector-muto-stack.json
BASE64=$(cat "$STACK" | base64 -w0)
jq --arg d "$BASE64" '.spec.components[0].properties.data=$d' apriltag-tracking-solution.json > /tmp/new-solution.json
mv /tmp/new-solution.json apriltag-tracking-solution.json
```

### Step 4: Deploy Instance
Create the Instance to bind the Solution to your target:

```bash
./define-instance.sh april-tag-robot/apriltag-tracking-instance.json
```

### Step 5: Verify Deployment
**ROS 2 Environment** (in container or host with ROS setup):
```bash
# Check AprilTag topics are available
ros2 topic list | grep apriltag

# Verify detection output
ros2 topic echo /apriltag/detections --once

# Monitor container status
ros2 node list | grep apriltag
```

**Symphony Environment**:
```bash
# Check instance status
curl -s http://localhost:8082/v1alpha2/instances | \
  jq '.items[] | select(.metadata.name|test("apriltag")) | {name: .metadata.name, phase: .status.phase}'

# View deployment details
curl -s http://localhost:8082/v1alpha2/instances/apriltag-tracking-instance | jq '.status'
```

### Step 6: Update and Rollout (Optional)
To demonstrate OTA updates, create a modified stack (e.g., add a second tag family):

1. Edit `apriltag-detector-muto-stack.json` to add another detector node
2. Create a new Solution version (`apriltag-tracking-solution-v2.json`)
3. Submit as a canary update:

```bash
# Create solution v2
./define-solution.sh april-tag-robot/apriltag-tracking-solution-v2.json

# Deploy to specific target (canary)
./define-instance.sh april-tag-robot/apriltag-tracking-instance-v2.json
```

## Monitoring and Telemetry

### Key Topics
Monitor these ROS 2 topics to verify AprilTag detection:

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/apriltag/detections` | `apriltag_msgs/AprilTagDetectionArray` | Core detection results with tag IDs and poses |
| `/apriltag/images/debug` | `sensor_msgs/Image` | Visualization with detection overlays (if enabled) |
| `/camera/image_raw` | `sensor_msgs/Image` | Input camera stream |
| `/apriltag/detections/compressed` | `CompressedImage` | Compressed debug visualization |

### Sample Detection Output
```bash
ros2 topic echo /apriltag/detections --once
```
Expected structure:
```yaml
detections:
- id: 1
  family: tag36h11
  center: {x: 320.5, y: 240.2}
  corners: [{x: 315.0, y: 235.0}, {x: 326.0, y: 235.0}, ...]
  pose:
    position: {x: 0.12, y: -0.05, z: 0.8}
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
```

## Troubleshooting

### Common Issues

**No detections appearing**:
```bash
# Check if camera is publishing
ros2 topic hz /camera/image_raw

# Verify detector node is running
ros2 node list | grep apriltag

# Check parameter configuration
ros2 param get /apriltag_container/apriltag_detector_node tag_family
```

**Deployment failing**:
```bash
# Check Symphony instance status
curl -s http://localhost:8082/v1alpha2/instances/apriltag-tracking-instance | jq '.status.observedState'

# View Muto agent logs
ros2 topic echo /muto/logs --once

# Check container health
docker ps | grep apriltag  # or podman ps
```

**Performance issues**:
- Reduce image resolution in camera driver
- Adjust `tag_decimate` parameter (trade accuracy for speed)
- Verify adequate CPU resources for container

### Validation Checklist

| Step | Command | Expected Result |
|------|---------|-----------------|
| Symphony API available | `curl http://localhost:8082/v1alpha2/greetings` | "Hello from Symphony..." |
| Target registered | `curl -s http://localhost:8082/v1alpha2/targets \| jq '.[].metadata.name'` | Shows target name |
| Instance deployed | `curl -s http://localhost:8082/v1alpha2/instances \| grep apriltag` | Instance exists |
| Camera publishing | `ros2 topic hz /camera/image_raw` | Shows frame rate |
| Detector active | `ros2 node list \| grep apriltag` | Detector node listed |
| Detections streaming | `ros2 topic echo /apriltag/detections --once` | Detection data appears |

## Advanced Configuration

### Update Policies
Configure deployment policies in Symphony for production scenarios:

```json
{
  "updatePolicy": {
    "canary": {
      "percentage": 10,
      "conditions": ["battery_state=docked", "cpu_usage<80%"]
    },
    "rollback": {
      "triggers": ["detection_fps<5", "error_rate>0.1"]
    }
  }
}
```

### Multi-Camera Setup
For robots with multiple cameras, create separate detector instances:

```json
{
  "node": [
    {
      "package": "apriltag_detector",
      "executable": "apriltag_detector_node", 
      "name": "front_camera_detector",
      "remappings": [{"from": "/image", "to": "/front_camera/image_raw"}]
    },
    {
      "package": "apriltag_detector",
      "executable": "apriltag_detector_node",
      "name": "rear_camera_detector", 
      "remappings": [{"from": "/image", "to": "/rear_camera/image_raw"}]
    }
  ]
}
```

## Next Steps

### Extensions
- **Depth Integration**: Add pose refinement using depth cameras
- **Tracking**: Implement persistent tag tracking across frames
- **Fleet Coordination**: Share tag detections between robots via DDS
- **Performance**: GPU-accelerated detection for real-time applications

### Production Considerations
- Configure signed container images for security compliance
- Implement proper tag size calibration procedures  
- Add detection confidence thresholds and filtering
- Set up centralized logging and metrics collection

## References
- [AprilTag Detector ROS 2 Package](https://github.com/ros-misc-utilities/apriltag_detector)
- [AprilTag Library Documentation](https://april.eecs.umich.edu/software/apriltag)
- [Symphony OTA Updates Guide](https://github.com/eclipse-symphony/symphony/blob/main/docs/README.md)
- [Muto Architecture Documentation](../../README.md)

---
This sample demonstrates basic AprilTag detection in a fleet management context. Build upon this foundation with tracking, multi-sensor fusion, and advanced deployment policies as your use case evolves.

