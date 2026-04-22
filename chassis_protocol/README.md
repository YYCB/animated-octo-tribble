# chassis_protocol

ROS2 chassis communication protocol HAL for differential-drive and mecanum-drive
robot platforms.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 Application                       │
│   /cmd_vel  /odom  /battery_state  /chassis_status  /diag  │
└───────────────────────┬─────────────────────────────────────┘
                        │ ROS2 topics
┌───────────────────────▼─────────────────────────────────────┐
│                    chassis_ros2::ChassisNode                 │
│              (ros2_adapter/chassis_node.hpp/.cpp)            │
└───────────────────────┬─────────────────────────────────────┘
                        │ IChassisController
┌───────────────────────▼─────────────────────────────────────┐
│   DifferentialController  │  MecanumController              │
│   (clamps, HAL subclass)  │  (clamps, HAL subclass)         │
├───────────────────────────┴─────────────────────────────────┤
│                      ChassisHal                              │
│      binary frame encoding / decoding / reconnect loop       │
├─────────────────────────────────────────────────────────────┤
│   VersionNegotiator  │  StreamDecoder  │  FrameCodec         │
│   (handshake FSM)    │  (byte-stream)  │  (CRC16-CCITT)      │
└──────────────────────┬──────────────────────────────────────┘
                       │ ITransport
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
   TcpTransport   UdpTransport  Rs485Transport
   (POSIX TCP)   (POSIX UDP)   (termios 8N1)
```

## Frame Format

```
[MAGIC:2B=0xAA55][VERSION:1B][CMD_ID:1B][LEN:2B][PAYLOAD:NB][CRC16:2B]
```

| Field   | Size | Description                                  |
|---------|------|----------------------------------------------|
| MAGIC   | 2B   | Fixed `0xAA 0x55`                            |
| VERSION | 1B   | Protocol version (current: `0x01`)           |
| CMD_ID  | 1B   | Command identifier (see table below)         |
| LEN     | 2B   | Payload length in bytes, little-endian       |
| PAYLOAD | NB   | Command-specific binary payload              |
| CRC16   | 2B   | CRC16-CCITT over VERSION+CMD_ID+LEN+PAYLOAD  |

### Command IDs

| ID     | Name             | Direction   |
|--------|------------------|-------------|
| `0x01` | HANDSHAKE_REQ    | Host → MCU  |
| `0x02` | HANDSHAKE_RESP   | MCU → Host  |
| `0x10` | VELOCITY_CMD     | Host → MCU  |
| `0x11` | LIGHT_CMD        | Host → MCU  |
| `0x12` | EMERGENCY_STOP   | Host → MCU  |
| `0x13` | WHEEL_CMD        | Host → MCU  |
| `0x20` | CHASSIS_STATUS   | MCU → Host  |
| `0x21` | ODOMETRY         | MCU → Host  |
| `0x22` | BATTERY_INFO     | MCU → Host  |
| `0xFF` | ACK              | Both        |

## Quick Start

### Building with colcon

```bash
cd ~/ros2_ws/src
git clone <repo-url> chassis_protocol
cd ~/ros2_ws
colcon build --packages-select chassis_protocol
source install/setup.bash
```

### Running the chassis node

```bash
ros2 run chassis_protocol chassis_node \
  --ros-args \
  -p transport_type:=tcp \
  -p host:=192.168.1.100 \
  -p port:=8899 \
  -p chassis_type:=differential
```

For a mecanum chassis over RS485:

```bash
ros2 run chassis_protocol chassis_node \
  --ros-args \
  -p transport_type:=rs485 \
  -p device:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p chassis_type:=mecanum
```

### Running tests

```bash
colcon test --packages-select chassis_protocol
colcon test-result --verbose
```

## ROS2 Parameters

| Parameter             | Type   | Default         | Description                          |
|-----------------------|--------|-----------------|--------------------------------------|
| `transport_type`      | string | `"tcp"`         | `tcp`, `udp`, or `rs485`             |
| `host`                | string | `192.168.1.100` | Remote host (TCP/UDP only)           |
| `port`                | int    | `8899`          | Remote port (TCP/UDP only)           |
| `device`              | string | `/dev/ttyUSB0`  | Serial device path (RS485 only)      |
| `baud_rate`           | int    | `115200`        | Serial baud rate (RS485 only)        |
| `chassis_type`        | string | `"differential"`| `differential` or `mecanum`          |
| `reconnect_interval_s`| double | `5.0`           | Reconnect check interval in seconds  |
| `frame_id`            | string | `"odom"`        | Odometry message frame ID            |
| `child_frame_id`      | string | `"base_link"`   | Odometry child frame ID              |

## ROS2 Topics

| Topic            | Type                                    | Direction   |
|------------------|-----------------------------------------|-------------|
| `/cmd_vel`       | `geometry_msgs/msg/Twist`               | Subscribed  |
| `/odom`          | `nav_msgs/msg/Odometry`                 | Published   |
| `/battery_state` | `sensor_msgs/msg/BatteryState`          | Published   |
| `/chassis_status`| `std_msgs/msg/String` (JSON)            | Published   |
| `/diagnostics`   | `diagnostic_msgs/msg/DiagnosticArray`   | Published   |

## Payload Wire Formats

All multi-byte integers are little-endian.

### VELOCITY_CMD (0x10) — 36 bytes

| Offset | Size | Field        |
|--------|------|--------------|
| 0      | 4    | seq          |
| 4      | 8    | vx (double)  |
| 12     | 8    | vy (double)  |
| 20     | 8    | omega (double)|
| 28     | 8    | timestamp_us |

### LIGHT_CMD (0x11) — 12 bytes

| Offset | Size | Field           |
|--------|------|-----------------|
| 0      | 1    | mode (uint8)    |
| 1      | 1    | r               |
| 2      | 1    | g               |
| 3      | 1    | b               |
| 4      | 8    | blink_hz (double)|

### EMERGENCY_STOP (0x12) — variable

| Offset | Size     | Field             |
|--------|----------|-------------------|
| 0      | 2        | reason_len        |
| 2      | reason_len | reason (UTF-8)  |

### WHEEL_CMD (0x13) — 13 + 8×N bytes

Per-wheel target angular velocity command. Host performs inverse kinematics; MCU only runs per-wheel PID loops.

| Offset | Size   | Field                                               |
|--------|--------|-----------------------------------------------------|
| 0      | 4      | seq                                                 |
| 4      | 8      | timestamp_us                                        |
| 12     | 1      | n_wheels                                            |
| 13     | 8×N    | wheel_rads[0..N-1] (rad/s, little-endian double)   |

**Wheel order:** Differential → [0] left, [1] right. Mecanum → [0] FL, [1] FR, [2] RL, [3] RR.

### CHASSIS_STATUS (0x20) — variable

| Offset | Size       | Field          |
|--------|------------|----------------|
| 0      | 4          | seq            |
| 4      | 8          | timestamp_us   |
| 12     | 4          | error_code     |
| 16     | 2          | error_msg_len  |
| 18     | msg_len    | error_msg      |

### ODOMETRY (0x21) — 668 bytes

| Offset | Size | Field                       |
|--------|------|-----------------------------|
| 0      | 4    | seq                         |
| 4      | 8    | timestamp_us                |
| 12     | 8    | x                           |
| 20     | 8    | y                           |
| 28     | 8    | z                           |
| 36     | 8    | qx                          |
| 44     | 8    | qy                          |
| 52     | 8    | qz                          |
| 60     | 8    | qw                          |
| 68     | 8    | vx                          |
| 76     | 8    | vy                          |
| 84     | 8    | omega                       |
| 92     | 288  | cov_pose (36 × double)      |
| 380    | 288  | cov_twist (36 × double)     |

### BATTERY_INFO (0x22) — variable

| Offset | Size         | Field          |
|--------|--------------|----------------|
| 0      | 8            | voltage        |
| 8      | 8            | current        |
| 16     | 8            | percentage     |
| 24     | 8            | temperature    |
| 32     | 1            | is_charging    |
| 33     | 1            | num_cells      |
| 34     | num_cells × 8 | cell_voltages |

## Handshake Protocol

1. On connect, the host sends `HANDSHAKE_REQ` with its protocol version, list of
   supported CMD IDs, and a config hash.
2. The MCU responds with `HANDSHAKE_RESP` indicating whether it accepts the
   version, and what chassis type it is.
3. If rejected, the host marks the negotiator as `FAILED`. It will not send
   further commands.
4. If no response is received within 2 seconds, the host retries up to 3 times
   before giving up.

## Auto-Reconnect

`ChassisHal` runs a background `reconnectLoop` thread that:
- Monitors `transport_->isConnected()` and `negotiator_.isNegotiated()`.
- On disconnection, waits for an exponential backoff period (500 ms → 1 s → 2 s
  … up to 30 s) then attempts to re-connect the transport and redo the handshake.
- `TcpTransport` also has its own internal reconnect loop within the receive
  thread.

## Extending

### Adding a new chassis type

1. Subclass `ChassisHal` (see `DifferentialController` for a minimal example).
2. Override `sendVelocity()` to apply kinematic constraints.
3. Override `getInfo()` to return the correct `ChassisType` enum value.
4. Register the new type in `ChassisNode::initController()`.

### Adding a new transport

1. Implement `ITransport` (send, setReceiveCallback, connect, disconnect,
   isConnected, getLastError).
2. Add a new `TransportType` enum value.
3. Add a case to `ITransport::create()` in `transport/itransport.cpp`.

## License

Apache-2.0 — see the `package.xml` for details.
