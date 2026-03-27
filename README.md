# HamFly API

A C API for the HamFly flight control system. This is version 2 of the FreeflyAPI, preserving backward compatibility with the existing QX protocol while adding enhanced packet sending and receiving capabilities.

## Overview

HamFly is designed to work with PSoC 5 LP microcontrollers using PSoC Creator 4.4, while maintaining flexibility for future microprocessor support.

## Project Structure

```
HamflyAPI/
├── src/                    # Core portable API code
│   ├── hamfly.h           # Main API entry point
│   ├── hamfly_core_*.h/c  # Core functionality (HAL, control, telemetry, gimbal)
│   ├── hamfly_comm_*.h/c  # Communication utilities (ring buffer, TX buffer)
│   └── hamfly_qx_*.h/c    # QX protocol implementation (backward compatible)
└── platform/              # Platform-specific implementations
    └── hamfly_platform_psoc5.h  # PSoC5 LP implementation
```

## Getting Started

### PSoC Creator Integration

1. Copy all files from the `src/` directory into your PSoC Creator project
2. Include `hamfly_core_hal.h` in your project to access the HAL interface
3. Use the PSoC5-specific implementation from `platform/hamfly_platform_psoc5.h`

### File Organization

- **src/**: Contains all portable core API code. These files implement the HamFly API interface.
- **platform/**: Contains platform-specific implementations. Currently includes PSoC5 support.
  - `hamfly_platform_psoc5.h`: Implements the HAL interface for PSoC5 LP

## Features

- **QX Protocol Compatibility**: Preserves existing QX protocol support for drop-in backward compatibility
- **Packet Communication**: Dedicated packet sending and receiving capabilities
- **HAL Interface**: Clean hardware abstraction layer in `hamfly_core_hal.h`
- **Modular Design**: Separate modules for control, telemetry, gimbal, and communication

## Adding New Platforms

To add support for a new microprocessor:

1. Create `platform/hamfly_platform_{microprocessor}.h`
2. Implement the HAL interface defined in `src/hamfly_core_hal.h`
3. Include the appropriate platform implementation in your build system

## License

Copyright 2026 Hamish Johnson
Quantum Information Systems Lab, SFU Physics

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for the full license text.

The QX protocol files (`hamfly_qx_protocol.*`, `hamfly_qx_app.*`) are derived from the
Freefly QX Protocol (Copyright 2017 Freefly Systems), also licensed under Apache 2.0.

## Contributing

This API is designed for active development. Feel free to extend and modify as needed for your use case.
