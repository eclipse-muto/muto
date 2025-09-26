#!/bin/bash

# Script to create a base64 encoded tar archive of a directory's contents and save it as JSON
# Usage: ./create_archive.sh <input_directory> <output_directory>

# Input validation
if [ $# -ne 2 ]; then
    echo "Usage: $0 <input_directory> <output_directory>"
    echo "  <input_directory>: Directory to archive"
    echo "  <output_directory>: Directory where the JSON file will be written"
    exit 1
fi

INPUT_DIR="$1"
OUTPUT_DIR="$2"

# Validate input directory
if [ ! -d "$INPUT_DIR" ]; then
    echo "Error: $INPUT_DIR is not a directory"
    exit 1
fi

# Validate output directory
if [ ! -d "$OUTPUT_DIR" ]; then
    echo "Error: $OUTPUT_DIR is not a directory"
    exit 1
fi



# Extract solution name from filename (remove path and .json extension)
ROOT_NAME=$(basename "$INPUT_DIR")
MANIFEST_NAME="${ROOT_NAME}-archive"

# Function to encode base64 from stdin
encode_base64() {
    # Read from stdin and encode to base64
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS (BSD base64)
        base64
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]] || [[ -n "$WSL_DISTRO_NAME" ]] || [[ -n "$IS_WSL" ]] || [[ "$(uname -r)" == *Microsoft* ]] || [[ "$(uname -r)" == *microsoft* ]]; then
        # WSL/Windows environments
        base64 | tr -d '\n'
    else
        # Linux/Unix (GNU base64)
        base64 -w 0
    fi
}

# Create tar archive of all contents and encode in base64
STACK_DATA_BASE64=$(tar -C $INPUT_DIR -czf - . | encode_base64)

# SHA256 checksum of the base64 encoded data
CHECKSUM=$(echo -n "$STACK_DATA_BASE64" | sha256sum | awk '{print $1}')
echo "SHA256 Checksum: $CHECKSUM"

# Find the relative path of the first file that ends with launch.py
LAUNCH_FILE=$(cd "$INPUT_DIR" && find . -type f -name "*.launch.py" | head -n 1 | sed 's|^\./||')
if [ -z "$LAUNCH_FILE" ]; then
    echo "Error: No launch.py file found in the directory"
    exit 1
fi
echo "Found launch file: $LAUNCH_FILE"

# Create JSON manifest
STACK_MANIFEST=$(cat << EOF
{
    "metadata": {
        "name": "${MANIFEST_NAME}",
        "description": "A simple talker-listener stack example using demo_nodes_cpp package.",
        "content_type": "stack/archive"
    },
    "launch": {
        "data": "${STACK_DATA_BASE64}",
        "properties": {
            "algorithm": "sha256",
            "checksum": "${CHECKSUM}",
            "launch_file": "${LAUNCH_FILE}",
            "command": "launch",
            "launch_args": [
                {
                    "name": "foo",
                    "default": "bar"
                }
            ],
            "ros_args": [],
            "flatten": true
        }
    }
}
EOF
)

# Write to JSON file in output directory
JSON_FILE="$OUTPUT_DIR/${MANIFEST_NAME}.json"
echo "$STACK_MANIFEST" > "$JSON_FILE"
echo "Created stack manifest JSON file: $JSON_FILE"