# Talker-Listener sample (Muto + Symphony)

This folder contains a small ROS 2 talker/listener stack used as a working example for Eclipse Muto orchestration and Eclipse Symphony solution/instance management.

This README describes:
- How to start the local Muto system (native ROS 2)
- How to start Symphony services (Docker Compose)
- How to register a solution and create an instance using the helper scripts
- Two ways to represent a stack: plain JSON (`stack/json`) and a packaged archive (`stack/archive`)
- How to create a new stack/archive with `create_archive.sh`

The commands below use ROS 2 **Humble** (Ubuntu 22.04). ROS 2 **Jazzy** (Ubuntu 24.04) works the same way: set `ROS_DISTRO=jazzy` instead of `humble` (see the first step), notes below point out the few differences.

## Prerequisites

- ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04) and a Muto workspace built from source, see [Building from Source](../../developer_guide/building_from_source.md)
- Docker with Docker Compose (`docker compose`) for Symphony. On Ubuntu 24.04 you can also use Podman with `podman-compose`
- `curl`, `jq`, and `base64` for the helper scripts (`sudo apt install -y curl jq coreutils`)

To run Muto from the pre-built container images instead of a native build, see the [Quick Start](../../user_guide/quick_start.md).

## Quick path — start Muto (locally)

If you already have a built workspace (`~/muto_ws`, see [Building from Source](../../developer_guide/building_from_source.md)) and want to run Muto locally (native ROS 2):

1. Select your ROS 2 distribution, then build and source the workspace:

   ```bash
   # Pick the distro installed on your machine: humble (Ubuntu 22.04) or jazzy (Ubuntu 24.04)
   export ROS_DISTRO=humble   # or: jazzy
   source /opt/ros/$ROS_DISTRO/setup.bash

   # Build (muto_paho_mqtt_vendor is nested in the agent, so it needs --base-paths)
   cd ~/muto_ws
   colcon build --base-paths src src/muto/src/agent/muto_paho_mqtt_vendor --symlink-install

   # Source the install overlay
   source install/setup.bash
   ```

2. Launch the main Muto system from the repository root (example args used in docs and tests):

   ```bash
   cd ~/muto_ws/src/muto
   ros2 launch launch/muto.launch.py \
       vehicle_namespace:=org.eclipse.muto.test \
       vehicle_name:=test-robot-debug \
       enable_symphony:=true \
       log_level:=INFO
   ```

This will start agent/composer/plugins and — when enabled — the Symphony provider so the device can talk to the Symphony control plane. Look for `Successfully registered target 'test-robot-debug'` in the output.

Notes
- Keep this terminal open. In every other terminal used below, source ROS 2 and the workspace first: `source /opt/ros/$ROS_DISTRO/setup.bash && source ~/muto_ws/install/setup.bash`.
- Verify the Muto nodes with `ros2 node list | grep muto`.

## Start Symphony (Docker Compose)

Symphony (API + Portal + Mosquitto) can be started from `docs/samples/symphony` using Docker Compose:

1. From repository root:

   ```bash
   cd docs/samples/symphony
   docker compose up -d
   # Ubuntu 24.04 with Podman: podman-compose up -d
   ```

2. Verify services:

   ```bash
   # Symphony API
   curl http://localhost:8082/v1alpha2/greetings

   # Symphony Portal GUI
   xdg-open http://localhost:3000  # or visit in your browser

   # MQTT broker
   # mosquitto: tcp://localhost:1883
   ```

If you need to stop the services:

   ```bash
   docker compose down
   # Ubuntu 24.04 with Podman: podman-compose down
   ```

Notes
- The `docker-compose.yaml` in `docs/samples/symphony` includes a ready-to-run Symphony API, Symphony Portal, and an Eclipse Mosquitto broker.
- Symphony keeps its state in memory: `docker compose down` followed by `docker compose up -d` gives you a clean Symphony without solutions, instances and targets. Muto registers its target only at startup, so launch Muto again after restarting Symphony.

## Automated demo with run-demo.sh

For a streamlined end-to-end experience, use the `run-demo.sh` script in this folder. It automates the process of verifying Symphony availability, waiting for Muto nodes to be ready, defining the archive-based solution, and creating an instance.

### Prerequisites for run-demo.sh

- Symphony services must already be running (see "Start Symphony (Docker Compose)" section).
- Muto must be launched (see "Quick path — start Muto (locally)" section).
- Required tools: `curl`, `jq`, `base64`, `ros2` (run the script in a terminal where ROS 2 and the workspace are sourced).

### How to run run-demo.sh

From the repository root:

```bash
cd docs/samples/talker-listener
./run-demo.sh
```

### What run-demo.sh does

1. **Checks prerequisites**: Verifies that required commands (`curl`, `jq`, `base64`, `ros2`) are available.
2. **Waits for Symphony API**: Polls `http://localhost:8082/v1alpha2/greetings` until the API responds (up to 60 seconds).
3. **Waits for Muto nodes**: Checks `ros2 node list` for the presence of required Muto nodes (up to 60 seconds):
   - `/muto/agent`
   - `/muto/commands_plugin`
   - `/muto/compose_plugin`
   - `/muto/core_twin`
   - `/muto/gateway`
   - `/muto/launch_plugin`
   - `/muto/muto_composer`
   - `/muto/muto_symphony_provider`
   - `/muto/provision_plugin`
4. **Defines the solution**: Uses `define-solution.sh` to post `talker-listener-xarchive.json` to Symphony.
5. **Creates the instance**: Uses `define-instance.sh` to create an instance with `talker-listener-xarchive-instance.json`.
6. **Provides next steps**: Prints commands to check Symphony Portal/API, verify ROS topics, and clean up.

If any step fails (e.g., Symphony not running, Muto nodes not appearing), the script exits with an error message.

After a few seconds the talker and listener started by Muto are running; verify them as described at the end of the next section.

## Define a solution (stack) and create an instance

The repo includes helper scripts in `docs/samples/symphony` that use the Symphony API to create solutions and instances:

- `define-solution.sh` — posts a solution to Symphony. Usage: `./define-solution.sh <stack-json-file>`
- `define-instance.sh` — creates an instance (binds a solution to a target). Usage: `./define-instance.sh <instance-json-file>`
- `delete-instance.sh` — deletes an instance. Usage: `./delete-instance.sh <instance-json-file>`

Examples (recommended sequence):

1. Start Symphony (see previous section) and ensure it responds on `http://localhost:8082`.

2. Create a solution describing the stack. From `docs/samples/symphony` run one of the options below.

Option A: Use the plain JSON stack (stack/json)

- The talker/listener JSON stack: `../talker-listener/talker-listener-json.json`.
- Example (run from `docs/samples/symphony`):

   ```bash
   ./define-solution.sh ../talker-listener/talker-listener-json.json
   ```

Option B: Use the packaged stack archive (stack/archive)

- A stack archive JSON is a manifest that contains base64-encoded tar.gz data (content_type: `stack/archive`). Example file: `talker-listener-xarchive.json`.
- Example (run from `docs/samples/symphony`):

   ```bash
   ./define-solution.sh ../talker-listener/talker-listener-xarchive.json
   ```

Both approaches create a Symphony solution that can later be instantiated against a target.

3. Create an instance to bind the solution to a registered target (robot). The sample instance files are in this folder:

- `talker-listener-json-instance.json` — uses the JSON stack solution
- `talker-listener-xarchive-instance.json` — uses the archive-based solution

Example (run from `docs/samples/symphony`):

   ```bash
   ./define-instance.sh ../talker-listener/talker-listener-json-instance.json
   ```

or for the archive-based solution:

   ```bash
   ./define-instance.sh ../talker-listener/talker-listener-xarchive-instance.json
   ```

A target runs one instance at a time: if the target already has an instance (for example from `run-demo.sh`), creating another one fails with `HTTP status: 400`. Use either Option A or Option B, or start from a clean Symphony (see the notes in "Start Symphony").

After creating an instance, verify from Symphony API (the API requires a token):

   ```bash
   TOKEN=$(curl -s -X POST -H "Content-Type: application/json" \
        -d '{"username":"admin","password":""}' \
        http://localhost:8082/v1alpha2/users/auth | jq -r .accessToken)

   curl -s -H "Authorization: Bearer $TOKEN" http://localhost:8082/v1alpha2/instances | jq '.[].metadata.name'
   curl -s -H "Authorization: Bearer $TOKEN" http://localhost:8082/v1alpha2/solutions | jq '.[].metadata.name'
   ```

And on the robot / Muto side check whether the stack launched (in a terminal with ROS 2 sourced):

   ```bash
   ros2 node list | grep -E 'talker|listener'
   # Expected output:
   # /listener
   # /talker
   ros2 topic echo --once /chatter
   # data: 'Hello World: <n>'
   ```

## Creating a new stack archive (packaging a directory)

If you want to create a `stack/archive` manifest from a directory containing a launch-based ROS stack, use `create_archive.sh` located in this folder.

Usage

1. Prepare a directory with the stack contents (for example the included `sample-stack/` directory which contains a `launch/` file).

2. Run the script from the repo or this folder. The script expects two arguments: `<input_directory>` and `<output_directory>`.

Example (create manifest in the same folder):

   ```bash
   cd docs/samples/talker-listener
   ./create_archive.sh sample-stack .
   ```

This will produce a JSON file named like `<input_directory>-archive.json` containing:
- `metadata` (name/description/content_type/version)
- `launch.data`: base64-encoded tar.gz of the `sample-stack/` directory
- `launch.properties`: checksum, the launch file, flatten flag and other metadata

The generated manifest sets `metadata.version` to `1`: `define-solution.sh` names the solution `<file name>-v-<metadata.version>`, and an instance refers to it as `<file name>:<version>`.

You can then publish the generated file to Symphony with `define-solution.sh`:

   ```bash
   cd ../symphony
   ./define-solution.sh ../talker-listener/sample-stack-archive.json
   # Expected: ✅ Solution 'sample-stack-archive-v-1' created successfully!
   ```

## Files in this folder

- `talker-listener-json.json` — stack definition in plain JSON (content_type: `stack/json`)
- `talker-listener-json-instance.json` — instance file for the JSON stack
- `talker-listener-xarchive.json` — stack archive manifest (base64-encoded tar data)
- `talker-listener-xarchive-instance.json` — instance file for the archive stack
- `sample-stack/` — example directory that can be packaged with `create_archive.sh` (produces `sample-stack-archive.json`)
- `run-demo.sh` — automated end-to-end demo script (requires Symphony and Muto running)
- `create_archive.sh` — helper script that packages a directory into a `stack/archive` JSON manifest

## Troubleshooting & tips

- Authentication: the helper scripts assume Symphony API is running locally and accept the default admin login (empty password). Adjust `SYMPHONY_API_URL` and credentials in the scripts if your environment differs.
- Tools: `jq` and `curl` are required by the helper scripts. Install them with your OS package manager.
- If the talker/listener nodes do not appear after instance creation, check `ros2 node list`, the Muto logs, and ensure the Symphony provider in Muto is enabled (`enable_symphony:=true` when launching the Muto system).

## Summary

This folder demonstrates both the minimal JSON stack format and the archive-based stack format. Use `define-solution.sh` to publish stacks to Symphony and `define-instance.sh` to bind them to targets (robots). Use `create_archive.sh` to create new archive-type manifests from a directory containing a ROS launch stack.
