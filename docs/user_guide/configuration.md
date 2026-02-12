# Configuration

Eclipse Muto is configured through two layers: a **YAML config file** (`config/muto.yaml`) that sets topic names, package lists, and other static settings, and **environment variables** that override connection and credential parameters at launch time.

## Environment Variables

The launch file (`launch/muto.launch.py`) reads the following environment variables via `os.getenv()` and passes them as ROS node parameters to every Muto node. Each variable has a default value that points to the public sandbox environment, so Muto works out of the box without setting anything.

### Twin / Core Connection

These configure the connection to the [Eclipse Ditto](https://eclipse.dev/ditto) digital twin server.

| Variable | ROS Parameter | Default | Description |
|---|---|---|---|
| `MUTO_TWIN_URL` | `twin_url` | `https://ditto:ditto@sandbox.composiv.ai` | Base URL of the Ditto twin API |
| `MUTO_TWIN_USER` | `twin_user` | `ditto` | Username for Ditto HTTP basic auth |
| `MUTO_TWIN_PASSWORD` | `twin_password` | `ditto` | Password for Ditto HTTP basic auth |
| `MUTO_HOST` | `host` | `sandbox.composiv.ai` | Hostname for MQTT broker and connectivity checks |
| `MUTO_PORT` | `port` | `1883` | MQTT broker port (coerced to integer) |

### Symphony / Orchestration

These configure the connection to [Eclipse Symphony](https://github.com/eclipse-symphony/symphony) for fleet orchestration.

| Variable | ROS Parameter | Default | Description |
|---|---|---|---|
| `MUTO_SYMPHONY_USER` | `symphony_user` | `admin` | Symphony authentication username |
| `MUTO_SYMPHONY_PASSWORD` | `symphony_password` | *(empty)* | Symphony authentication password |
| `MUTO_SYMPHONY_HOST` | `symphony_host` | `localhost` | Symphony MQTT broker hostname |
| `MUTO_SYMPHONY_PORT` | `symphony_port` | `1883` | Symphony MQTT broker port (coerced to integer) |
| `MUTO_SYMPHONY_API_URL` | `symphony_api_url` | `http://localhost:8082/v1alpha2/` | Symphony REST API base URL |
| `MUTO_SYMPHONY_BROKER_ADDRESS` | `symphony_broker_address` | `tcp://mosquitto:1883` | Full broker address used by the Symphony provider |

### Usage

Export any variable before launching to override the default:

```bash
export MUTO_TWIN_URL=https://my-ditto-instance.example.com
export MUTO_TWIN_USER=myuser
export MUTO_TWIN_PASSWORD=mypassword
export MUTO_HOST=my-ditto-instance.example.com

ros2 launch muto muto.launch.py vehicle_name:=my-vehicle
```

A `.env.example` file is provided at the repository root as a reference for all available variables with their sandbox defaults.

## Launch Arguments

In addition to environment variables, the launch file accepts the following arguments:

| Argument | Default | Description |
|---|---|---|
| `vehicle_name` | *(required)* | Unique vehicle identifier |
| `vehicle_namespace` | `org.eclipse.muto.sandbox` | Vehicle ID namespace |
| `muto_namespace` | `muto` | ROS namespace for all Muto nodes |
| `muto_config_file` | `config/muto.yaml` | Path to the YAML config file |
| `enable_symphony` | `true` | Enable/disable the Symphony MQTT provider (`true`/`false`) |
| `log_level` | `INFO` | Logging level for all nodes (`DEBUG`, `INFO`, `WARN`, `ERROR`) |

### Example

```bash
ros2 launch muto muto.launch.py \
  vehicle_name:=robot-01 \
  vehicle_namespace:=org.example.fleet \
  log_level:=DEBUG \
  enable_symphony:=false
```

## Precedence

Parameters are resolved in this order (last wins):

1. Defaults declared in `package.xml` / node code (`declare_parameter`)
2. Values from the YAML config file (`config/muto.yaml`)
3. Environment variable overrides (via `env_params` dict in the launch file)
4. Launch argument overrides (e.g. `vehicle_name:=...`)

## YAML Config File

The default config file at `config/muto.yaml` contains topic names, package ignore lists, command definitions, and other static settings that rarely change between environments. See that file for the full list of available ROS parameters.
