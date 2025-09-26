# Talker-Listener sample (Muto + Symphony)

This folder contains a small ROS 2 talker/listener stack used as a working example for Eclipse Muto orchestration and Eclipse Symphony solution/instance management.

This README describes:
- How to start the local Muto system (native or container/dev workflow)
- How to start Symphony services (docker compose)
- How to register a solution and create an instance using the helper scripts
- Two ways to represent a stack: plain JSON (`stack/json`) and a packaged archive (`stack/archive`)
- How to create a new stack/archive with `create_archive.sh`

## Prerequisites

- ROS 2 (Humble or later) and `colcon` if running Muto natively
- Docker and Docker Compose (or `docker compose`) for Symphony
- `curl`, `jq`, and `base64` available for the helper scripts

For a complete system setup and additional details see: `../../muto-quickstart.md` and the repository `README.md`.

## Quick path — start Muto (locally)

If you already have a built workspace and want to run Muto locally (native ROS 2):

1. Build and source your workspace (example):

   1. Build

      source /opt/ros/humble/setup.bash
      colcon build --symlink-install

   2. Source the install overlay

      source install/setup.bash

2. Launch the main Muto system (example args used in docs and tests):

   ```bash
   ros2 launch launch/muto.launch.py \
       vehicle_namespace:=org.eclipse.muto.test \
       vehicle_name:=test-robot-debug \
       enable_symphony:=true \
       log_level:=INFO
   ```

This will start agent/composer/plugins and — when enabled — the Symphony provider so the device can talk to the Symphony control plane.

Notes
- If you prefer running inside the development container described in `docs/development-container.md`, follow that doc to enter the container and run the same `ros2 launch` command after building.
- Verify nodes with `ros2 node list | grep muto` and topics with `ros2 topic list` / `ros2 topic echo /chatter`.

## Start Symphony (docker-compose)

Symphony (API + Portal + Mosquitto) can be started from `docs/samples/symphony` using Docker Compose:

1. From repository root:

   ```bash
   cd docs/samples/symphony
   docker compose up -d
   ```

2. Verify services:

   ```bash
   # Symphony API
   curl http://localhost:8082/v1alpha2/greetings

   # Symphony Portal GUI
   open http://localhost:3000  # or visit in your browser

   # MQTT broker
   # mosquitto: tcp://localhost:1883
   ```

If you need to stop the services:

   ```bash
   docker compose down
   ```

Notes
- The `docker-compose.yaml` in `docs/samples/symphony` includes a ready-to-run Symphony API, Symphony Portal, and an Eclipse Mosquitto broker.

## Automated demo with run-demo.sh

For a streamlined end-to-end experience, use the `run-demo.sh` script in this folder. It automates the process of verifying Symphony availability, waiting for Muto nodes to be ready, defining the archive-based solution, and creating an instance.

### Prerequisites for run-demo.sh

- Symphony services must already be running (see "Start Symphony (docker-compose)" section).
- Muto must be launched (see "Quick path — start Muto (locally)" section).
- Required tools: `curl`, `jq`, `base64`, `ros2`.

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


## Define a solution (stack) and create an instance

The repo includes helper scripts in `docs/samples/symphony` that use the Symphony API to create solutions and instances:

- `define-solution.sh` — posts a solution to Symphony. Usage: `./define-solution.sh <stack-json-file>`
- `define-instance.sh` — creates an instance (binds a solution to a target). Usage: `./define-instance.sh <instance-json-file>`
- `delete-instance.sh` — deletes an instance. Usage: `./delete-instance.sh <instance-json-file>`

Examples (recommended sequence):

1. Start Symphony (see previous section) and ensure it responds on `http://localhost:8082`.

2. Create a solution describing the stack. From `docs/samples/symphony` run one of the options below.

Option A: Use the plain JSON stack (stack/json)

- The talker/listener JSON stack: `../talker-listener-json.json` (or `../talker-listener/talker-listener-json.json` depending on where you execute the script).
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

After creating an instance, verify from Symphony API:

   ```bash
   curl -H "Content-Type: application/json" http://localhost:8082/v1alpha2/instances
   curl -H "Content-Type: application/json" http://localhost:8082/v1alpha2/solutions
   ```

And on the robot / Muto side check whether the stack launched:

   ```bash
   ros2 topic list
   ros2 topic echo /chatter
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
- `metadata` (name/description/content_type)
- `launch.data`: base64-encoded tar.gz of the `sample-stack/` directory
- `launch.properties`: checksum, the launch file, flatten flag and other metadata

You can then publish the generated file to Symphony with `define-solution.sh`:

   ```bash
   cd ../symphony
   ./define-solution.sh ../talker-listener/sample-stack-archive.json
   ```

## Files in this folder

- `talker-listener-json.json` — stack definition in plain JSON (content_type: `stack/json`)
- `talker-listener-json-instance.json` — instance file for the JSON stack
- `talker-listener-xarchive.json` — stack archive manifest (base64-encoded tar data)
- `talker-listener-xarchive-instance.json` — instance file for the archive stack
- `sample-stack-archive.json` — example pre-created archive manifest (created by `create_archive.sh`)
- `sample-stack/` — example directory that can be packaged with `create_archive.sh`
- `run-demo.sh` — automated end-to-end demo script (requires Symphony and Muto running)
- `create_archive.sh` — helper script that packages a directory into a `stack/archive` JSON manifest

## Troubleshooting & tips

- Authentication: the helper scripts assume Symphony API is running locally and accept the default admin login (empty password). Adjust `SYMPHONY_API_URL` and credentials in the scripts if your environment differs.
- Tools: `jq` and `curl` are required by the helper scripts. Install them with your OS package manager.
- If the Muto nodes do not appear after instance creation, check `ros2 node list`, the Muto logs, and ensure the Symphony provider in Muto is enabled (`enable_symphony:=true` when launching the Muto system).

## Summary

This folder demonstrates both the minimal JSON stack format and the archive-based stack format. Use `define-solution.sh` to publish stacks to Symphony and `define-instance.sh` to bind them to targets (robots). Use `create_archive.sh` to create new archive-type manifests from a directory containing a ROS launch stack.

If you want me to also add example commands to automatically register a target (target JSON/templates are in `docs/samples/symphony/target.json`) or to create a small troubleshooting checklist, I can add that next.