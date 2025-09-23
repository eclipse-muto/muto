REG ?= ghcr.io/eclipse-muto/muto
TAG ?= ros2-humble
GIT_SHA ?= $(shell git rev-parse --short HEAD)
DATE ?= $(shell date -u +%Y-%m-%dT%H:%M:%SZ)
CONTAINERFILE ?= container/Containerfile

.PHONY: build-amd64 build-arm64 push-amd64 push-arm64 manifest push-manifest run

build-amd64:
	podman build \
	  --arch amd64 \
	  --build-arg VCS_REF=$(GIT_SHA) \
	  --build-arg BUILD_DATE=$(DATE) \
	  -f $(CONTAINERFILE) \
	  -t $(REG):$(TAG)-amd64-$(GIT_SHA) .

build-arm64:
	podman build \
	  --arch arm64 \
	  --build-arg VCS_REF=$(GIT_SHA) \
	  --build-arg BUILD_DATE=$(DATE) \
	  -f $(CONTAINERFILE) \
	  -t $(REG):$(TAG)-arm64-$(GIT_SHA) .

push-amd64:
	podman push $(REG):$(TAG)-amd64-$(GIT_SHA)

push-arm64:
	podman push $(REG):$(TAG)-arm64-$(GIT_SHA)

manifest:
	podman manifest create $(REG):$(TAG) \
	  $(REG):$(TAG)-amd64-$(GIT_SHA) \
	  $(REG):$(TAG)-arm64-$(GIT_SHA)

push-manifest:
	podman manifest push --all $(REG):$(TAG) docker://$(REG):$(TAG)

run:
	podman run --rm -it \
	  -e MUTO_LAUNCH=/work/launch/muto.launch.py \
      -e MUTO_LAUNCH_ARGS="vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-debug enable_symphony:=true" \
	  -v $$(pwd)/launch:/work/launch:ro \
	  -v $$(pwd)/config:/work/config:ro \
	  --network host \
	  $(REG):$(TAG)
