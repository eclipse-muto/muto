#!/usr/bin/env bash
# Lightweight end-to-end demo wrapper for the talker-listener sample.
# Assumes Symphony is already running; waits for the API, verifies Muto nodes,
# then defines the solution and creates an instance.
# Usage: ./run-demo.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYMPHONY_DIR="$SCRIPT_DIR/../symphony"
EXAMPLE="talker-listener-xarchive"
SOLUTION_JSON="$SCRIPT_DIR/${EXAMPLE}.json"
INSTANCE_JSON="$SCRIPT_DIR/${EXAMPLE}-instance.json"

# Check requirements (we assume Symphony is already running)
for cmd in curl jq base64 ros2; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Required command not found: $cmd" >&2
    echo "Please install it and re-run the script." >&2
    exit 1
  fi
done

echo "Assuming Symphony services are already running."

# Wait for Symphony API to be available
API_URL="http://localhost:8082/v1alpha2/greetings"
echo "Waiting for Symphony API at $API_URL..."
for i in $(seq 1 60); do
  if curl -sS "$API_URL" >/dev/null 2>&1; then
    echo "Symphony API is up"
    break
  fi
  printf "."
  sleep 1
  if [ "$i" -eq 60 ]; then
    printf '\nTimed out waiting for Symphony API\n' >&2
    exit 2
  fi
done

# Note: solution/instance creation will be performed after verifying Muto nodes
echo "Symphony API is available. Will define solution & create instance after Muto nodes are verified."

# Wait for expected Muto nodes to appear
REQUIRED_NODES=(
  "/muto/agent"
  "/muto/commands_plugin"
  "/muto/compose_plugin"
  "/muto/core_twin"
  "/muto/gateway"
  "/muto/launch_plugin"
  "/muto/muto_composer"
  "/muto/muto_symphony_provider"
  "/muto/provision_plugin"
)

echo "Waiting for Muto nodes to appear (timeout 60s)..."
missing=()
for i in $(seq 1 60); do
  NODE_LIST=$(ros2 node list 2>/dev/null || true)
  missing=()
  for n in "${REQUIRED_NODES[@]}"; do
    if ! printf '%s\n' "$NODE_LIST" | grep -F -x "$n" >/dev/null 2>&1; then
      missing+=("$n")
    fi
  done
  if [ ${#missing[@]} -eq 0 ]; then
    echo "All Muto nodes are present"
    break
  fi
  printf "."
  sleep 1
  if [ "$i" -eq 60 ]; then
    printf '\nTimed out waiting for Muto nodes. Missing: %s\n' "${missing[*]}" >&2
    exit 3
  fi
done

# Define solution and create instance now that Muto nodes are present
echo "Defining solution using $SOLUTION_JSON"
"$SYMPHONY_DIR/define-solution.sh" "$SOLUTION_JSON"

# Give Symphony a moment to register solution
sleep 1

echo "Creating instance using $INSTANCE_JSON"
"$SYMPHONY_DIR/define-instance.sh" "$INSTANCE_JSON"

# Final status
echo ""
echo "---"
echo "Demo finished. Check Symphony instances and Muto:" 
echo "  http://localhost:3000 Symphony Portal"
echo "  http://localhost:8082/v1alpha2/instances Symphony API"
echo "On the robot Muto verify nodes/topics:"
echo "  ros2 node list | grep muto"
echo "  ros2 topic echo /chatter"

echo "Next steps:"
echo "1. To stop symphony, run:"
echo "cd $SYMPHONY_DIR && docker compose down"
echo "2. To delete the instance, run:"
echo "$SYMPHONY_DIR/delete-instance.sh $INSTANCE_JSON"
echo "3. To stop muto"
echo "ros2 daemon stop"

