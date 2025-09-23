# Unified Muto container (ROS 2 humble)

This image bundles all Muto packages (core, messages, agent, composer) plus repo `launch/` and `config/`. It defaults to ROS 2 humble and supports multi-arch builds (linux/amd64 and linux/arm64).

## Build (multi-arch) with Podman

```bash
# Single-arch local build
podman build \
  -f container/Containerfile \
  -t muto:ros2-humble .

# Multi-arch: build per-arch images and create a manifest
REG=ghcr.io/eclipse-muto/muto
TAG=ros2-humble
GIT_SHA=$(git rev-parse --short HEAD)
DATE=$(date -u +%Y-%m-%dT%H:%M:%SZ)

# Build amd64
podman build \
  --arch amd64 \
  --build-arg VCS_REF=$GIT_SHA \
  --build-arg BUILD_DATE=$DATE \
  -f container/Containerfile \
  -t $REG:${TAG}-amd64-$GIT_SHA \
  .

# Build arm64
podman build \
  --arch arm64 \
  --build-arg VCS_REF=$GIT_SHA \
  --build-arg BUILD_DATE=$DATE \
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
- Base image: `ros:humble` (Ubuntu 22.04)
- GPU: With Podman, prefer using `--hooks-dir`/`nvidia-container-toolkit` or rootful Podman with Nvidia support on your host. Alternatively, build a CUDA base variant and run with `--device` mappings.
