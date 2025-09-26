# talker-listener-ws

A ROS 2 Python workspace that pairs a talker and listener node on the `chatter` topic. The talker publishes a greeting once per second, and the listener logs every received message. Use this project as a minimal template for experimenting with ROS 2 publishers, subscribers, and launch files.

## Repository Layout
- `config/talker.yaml` &mdash; parameter file loaded by the launch script (currently defines `publish_frequency` and `message`).
- `launch/talker_listener.launch.py` &mdash; starts both nodes in a single process using the configuration above.
- `src/muto_talker/` &mdash; Python package that exposes the `muto_talker` node.
- `src/muto_listener/` &mdash; Python package that exposes the `muto_listener` node.

## Prerequisites
- A ROS 2 distribution (Foxy, Galactic, Humble, or Rolling) with Python support.
- `colcon` and `rosdep` installed for building and resolving dependencies.
- A sourced ROS 2 environment (`source /opt/ros/<distro>/setup.bash`).

## Build
```bash
# From the workspace root
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
You can scope the build to one package at a time (e.g. `--packages-select muto_talker`) while iterating.

## Run
After sourcing `install/setup.bash`:

```bash
# Run each node separately
ros2 run muto_talker muto_talker
ros2 run muto_listener muto_listener

# Launch both nodes together
ros2 launch --launch-file-path launch/talker_listener.launch.py
```
> Tip: If you prefer `ros2 launch muto_talker talker_listener.launch.py`, add the launch file to the `data_files` section of the package setup so it is installed under `share/muto_talker/launch`.

The talker publishes `std_msgs/msg/String` messages such as `"Hello SDV Hackathon Chapter III! 3"`; the listener prints each payload to its logger.

## Configuration
The launch file loads `config/talker.yaml`, which currently defines:
```yaml
/talker:
  ros__parameters:
    publish_frequency: 1.0
    message: "Hello, SDV Hackathon Chapter III!"
```
The node implementation still uses inline defaults (1 Hz and the string shown above). Update `muto_talker/muto_talker.py` to declare and read these parameters if you want runtime configurability.

## Testing & Linting
The packages include the standard ROS 2 Python linters.
```bash
colcon test --packages-select muto_talker muto_listener
```
This hooks `ament_flake8`, `ament_pep257`, and copyright checks.

## Next Steps
- Extend the talker to declare parameters and use `publish_frequency` and `message` at runtime.
- Add more message types or QoS policies to experiment with ROS 2 communication strength.
- Package and install launch files so they are discoverable via `ros2 launch` without an explicit path.
