# Changelog

All notable changes to this project will be documented in this file.

## [0.1.0] - 2025-01-01

### Added
- Initial chassis communication protocol HAL design
- Frame codec with CRC16-CCITT and StreamDecoder state machine
- TCP, UDP, and RS485 transport implementations
- Version negotiation handshake protocol
- ChassisHal base class implementing IChassisController
- DifferentialController with velocity clamping
- MecanumController with full 3-DOF velocity support
- ROS2 adapter node (ChassisNode) supporting cmd_vel, odom, battery_state topics
- Protobuf schema (chassis.proto) for protocol documentation
- Unit tests for frame codec and mock transport
- Fixtures directory with capture format documentation
- Comprehensive README with architecture, quick start, and guides

### Changed
- N/A (initial release)

### Fixed
- N/A (initial release)
