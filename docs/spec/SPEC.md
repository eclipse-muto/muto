# Project Requirements

This document captures the requirements for the Eclipse Muto project based on
the existing documentation and repository guidance. It is intended to be a
single, quick reference for what the system must do, how it fits together, and
what the environment must provide.

## Purpose and scope
- The project must provide a declarative orchestrator for managing ROS stacks on edge devices, enabling remote fleet orchestration with preserved ROS logic. Source: `docs/project_overview.md`, `docs/readme.md`
- The system must support adaptive, model-driven stack composition and remote management across distributed fleets. Source: `docs/readme.md`

## System architecture requirements
- The solution must include a cloud to edge flow with an Agent gateway, Composer for reconciliation and stack lifecycle, Core for digital twin services, and a Messages package for ROS interfaces. Source: `docs/project_overview.md`, `docs/reference/readme.md`
- The Agent must act as the protocol gateway and message router between cloud orchestration and on-device ROS systems. Source: `docs/reference/agent/readme.md`
- The Composer must orchestrate pipelines that validate, provision, and launch stacks through plugins. Source: `docs/reference/composer/readme.md`
- The Messages package must define ROS message and service contracts for inter-component communication. Source: `docs/reference/messages/readme.md`

## Functional requirements

### Declarative stack management
- The system must accept stack definitions in supported formats (JSON manifest and archive-based stacks) and validate them before execution. Source: `docs/reference/composer/readme.md`
- The system must support pipeline-driven execution with compose, provision, and launch plugins. Source: `docs/reference/composer/readme.md`
- Stack definitions must be serializable and transportable for remote orchestration. Source: `docs/project_overview.md`

### Fleet orchestration and cloud integration
- The system should be able to integrate with a cloud orchestration platform (like Eclipse Symphony) for target, solution, and instance workflows. Source: `docs/project_overview.md`, `docs/user_guide/running_examples.md`
- The system must integrate with a digital twin service (Eclipse Ditto) for device state synchronization. Source: `docs/project_overview.md`, `docs/readme.md`

### Protocols and messaging
- The Agent must support MQTT and HTTP communication, with an extensible path for additional protocols. Source: `docs/readme.md`, `docs/reference/agent/readme.md`
- Inter-component communication must use ROS 2 messages and services defined in the Messages package (for example, MutoAction, StackManifest, and plugin services). Source: `docs/reference/messages/readme.md`

### Dashboard and API requirements
- The backend must expose the API contract defined in `docs/api/openapi.yaml`, including read and write endpoints for fleet, rollout, release, desired state, and report data. Source: `docs/api/openapi.yaml`, `docs/frontend/backend_responsibilities.md`
- The backend must provide pagination, filtering, search, stable response envelopes, and idempotent writes. Source: `docs/frontend/backend_responsibilities.md`
- The system must implement rollout statuses, canary and wave stages, and gates as documented. Source: `docs/frontend/rollouts.md`
- Vehicle status codes and reason codes must align with the documented list. Source: `docs/frontend/vehicle_status.md`
- Vehicle timeline events must include the documented event types. Source: `docs/frontend/vehicle_events.md`
- Audit log actions must include documented actions for deployments and rollbacks. Source: `docs/frontend/audit_log.md`

## Deployment and environment requirements
- Supported development platforms are Ubuntu 20.04 and 22.04 with ROS 2 installed. Source: `docs/developer_guide/readme.md`, `docs/developer_guide/building_from_source.md`
- Running examples require ROS 2 Humble or later. Source: `docs/user_guide/running_examples.md`
- Building from source requires Python 3.10+ and colcon, rosdep, and standard ROS 2 build tooling. Source: `docs/developer_guide/building_from_source.md`

## Quality, security, and operational requirements
- Development must follow ROS 2 best practices, Python style guidelines, and include comprehensive type hints and documentation. Source: `docs/developer_guide/readme.md`
- The test strategy must use `colcon test` with an 80 percent coverage target and independent tests. Source: `docs/developer_guide/readme.md`
- The backend must emit audit logs for write actions and provide observability (structured logs, metrics, tracing, health checks). Source: `docs/frontend/backend_responsibilities.md`
- Security must include input validation and secure communication patterns with authentication and encryption support. Source: `docs/developer_guide/readme.md`, `docs/reference/agent/readme.md`

## Constraints and non-goals
- The system must avoid direct remote command execution and rely on declarative desired state instead. Source: `docs/frontend/backend_responsibilities.md`

## Open items to confirm
- Python version baseline is listed as 3.10+ for building from source, while the developer guide mentions 3.8+. Confirm the official minimum. Source: `docs/developer_guide/building_from_source.md`, `docs/developer_guide/readme.md`
