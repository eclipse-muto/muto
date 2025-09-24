# Unified Muto container (parameterized ROS 2 distribution)

This image bundles all Muto packages (core, messages, agent, composer) plus repo `launch/` and `config/`.

Default ROS 2 distribution: `humble` (Ubuntu 22.04 base).

Override at build time with the `ROS_DISTRO` build arg, e.g. `ROS_DISTRO=iron` or `ROS_DISTRO=jazzy` (ensure the base image/tag exists upstream and dependency package names are valid for that distro).

## Build (multi-arch) with Podman

```bash
# Single-arch local build (default humble)
podman build -f container/Containerfile -t muto:ros2-humble .

# Single-arch targeting a different ROS distro (example: iron)
podman build --build-arg ROS_DISTRO=iron -f container/Containerfile -t muto:ros2-iron .

# Multi-arch: build per-arch images and create a manifest
REG=ghcr.io/eclipse-muto/muto
ROS_DISTRO=humble   # change to iron/jazzy, etc.
TAG=ros2-$ROS_DISTRO
GIT_SHA=$(git rev-parse --short HEAD)
DATE=$(date -u +%Y-%m-%dT%H:%M:%SZ)

# Build amd64
podman build \
  --arch amd64 \
  --build-arg VCS_REF=$GIT_SHA \
  --build-arg BUILD_DATE=$DATE \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  -f container/Containerfile \
  -t $REG:${TAG}-amd64-$GIT_SHA \
  .

# Build arm64
podman build \
  --arch arm64 \
  --build-arg VCS_REF=$GIT_SHA \
  --build-arg BUILD_DATE=$DATE \
  --build-arg ROS_DISTRO=$ROS_DISTRO \
  -f container/Containerfile \
  -t $REG:${TAG}-arm64-$GIT_SHA \
  .

# Create and push a manifest list
podman manifest create $REG:${TAG} \
  $REG:${TAG}-amd64-$GIT_SHA \
  $REG:${TAG}-arm64-$GIT_SHA

# Push per-arch images and manifest
podman push $REG:${TAG}-amd64-$GIT_SHA
podman push $REG:${TAG}-arm64-$GIT_SHA
podman manifest push --all $REG:${TAG} docker://$REG:${TAG}

# Optionally create a moving tag pointing to the same manifest
podman manifest create $REG:${TAG}-latest \
  $REG:${TAG}-amd64-$GIT_SHA \
  $REG:${TAG}-arm64-$GIT_SHA
podman manifest push --all $REG:${TAG}-latest docker://$REG:${TAG}-latest
```

## Run

```bash
podman run --rm -it \
  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
  -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
  -v $(pwd)/launch:/work/launch:ro \
  -v $(pwd)/config:/work/config:ro \
  --network host \
  ghcr.io/eclipse-muto/muto:ros2-humble
```

- Override `MUTO_LAUNCH` to select an alternative launch file.
- Mount host `config/` and `launch/` for development or override at build time to bake them in.

## Notes
- Base image: `ros:${ROS_DISTRO}` (default `humble`, Ubuntu 22.04)
- GPU: With Podman, prefer using `--hooks-dir`/`nvidia-container-toolkit` or rootful Podman with Nvidia support on your host. Alternatively, build a CUDA base variant and run with `--device` mappings.
