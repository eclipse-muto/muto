REG ?= ghcr.io/eclipse-muto/muto
# Override ROS_DISTRO to build for a different ROS 2 distribution (e.g. iron, jazzy)
ROS_DISTRO ?= humble
TAG ?= ros2-$(ROS_DISTRO)
GIT_SHA ?= $(shell git rev-parse --short HEAD)
VERSION ?=  v0.42_build_1
DATE ?= $(shell date -u +%Y-%m-%dT%H:%M:%SZ)
CONTAINERFILE ?= container/Containerfile

.PHONY: build-amd64 build-arm64 push-amd64 push-arm64 manifest push-manifest run

build-all: build-amd64 build-arm64

build-amd64:
	podman build \
	  --arch amd64 \
	  --build-arg VCS_REF=$(GIT_SHA) \
	  --build-arg BUILD_DATE=$(DATE) \
	  --build-arg ROS_DISTRO=$(ROS_DISTRO) \
	  -f $(CONTAINERFILE) \
	  -t $(REG):$(TAG)-amd64-$(VERSION) .

build-arm64:
	podman build \
	  --arch arm64 \
	  --build-arg VCS_REF=$(GIT_SHA) \
	  --build-arg BUILD_DATE=$(DATE) \
	  --build-arg ROS_DISTRO=$(ROS_DISTRO) \
	  -f $(CONTAINERFILE) \
	  -t $(REG):$(TAG)-arm64-$(VERSION) .

push-amd64:
	podman push $(REG):$(TAG)-amd64-$(VERSION)

push-arm64:
	podman push $(REG):$(TAG)-arm64-$(VERSION)

manifest:
	podman manifest create $(REG):$(TAG)-$(VERSION) \
	  $(REG):$(TAG)-amd64-$(VERSION) \
	  $(REG):$(TAG)-arm64-$(VERSION)

push-manifest:
	podman manifest push --all $(REG):$(TAG)-$(VERSION) docker://$(REG):$(TAG)-$(VERSION)

run:
	podman run --rm -it \
	  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
      -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
	  -v $$(pwd)/launch:/work/launch:ro \
	  -v $$(pwd)/config:/work/config:ro \
	  --network host \
	  $(REG):$(TAG)-$(VERSION)
