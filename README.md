# HamFly API

A C API for the Freefly ecosystem developed for the Movi Pro. This is an unofficial version 2 of the FreeflyAPI, preserving backward compatibility with the existing QX protocol while adding enhanced packet sending and receiving capabilities.

## Overview

HamFly was designed to for integration PSoC 5 LP microcontrollers using PSoC Creator 4.4, while maintaining flexibility for future microprocessor support. Please read the original [FreeflyAPI documentation](<docs/old_Freefly_API/Freefly API Version 1.0.pdf>).

## Project Structure

```
HamflyAPI/
├── src/                   # Core portable API code
│   ├── hamfly.h           # Main API entry point
│   ├── hamfly_core_*.h/c  # Core functionality (HAL, control, telemetry, gimbal)
│   ├── hamfly_comm_*.h/c  # Communication utilities (ring buffer, TX buffer)
│   └── hamfly_qx_*.h/c    # QX protocol implementation (backward compatible)
└── platform/              # Platform-specific implementations
    └── hamfly_platform_psoc5.h  # PSoC5 LP implementation
```

## Getting Started

### PSoC Creator Integration

See [Adding New Platforms](#adding-new-platforms) to see available hardware interfaces, or to quickly develop your own.\
1. Copy all files from the `src/` directory into your PSoC Creator project
2. Include `hamfly_core_hal.h` in your project to access the HAL interface
3. Use the PSoC5-specific implementation from `platform/hamfly_platform_psoc5.h`

### File Organization

- **src/**: Contains all portable core API code. These files implement the HamFly API interface.
- **platform/**: Contains platform-specific implementations. Currently includes PSoC5 support.

## Features

- **QX Protocol Compatibility**: Preserves existing QX protocol support for drop-in backward compatibility
- **Packet Communication**: Dedicated packet sending and receiving capabilities
- **HAL Interface**: Clean hardware abstraction layer in `hamfly_core_hal.h`
- **Modular Design**: Separate modules for control, telemetry, gimbal, and communication

## Adding New Platforms

To add support for a new microprocessor:

1. Create `platform/hamfly_platform_{microprocessor}.h` (or your own naming)
2. Implement the HAL interface defined in `src/hamfly_core_hal.h`
3. Include the appropriate platform implementation in your build system

## License

Copyright 2026 Hamish Johnson
Quantum Information Systems Lab, SFU Physics

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for the full license text.

The QX protocol files (`hamfly_qx_protocol.*`, `hamfly_qx_app.*`) are derived from the
Freefly QX Protocol (Copyright 2017 Freefly Systems), also licensed under Apache 2.0.

## FreeflyAPI Behavioral Notes

These are behaviors of the MōVI Pro / FreeflyAPI protocol — not HamFlyAPI
bugs — that have cost integrators real design time. Documented here so the
next integrator doesn't rediscover them.

### Pan rebase on new input

The **first ABSOLUTE Pan command from a new input source** re-zeros the pan
reference to the current pan angle: an offset is applied such that the
current pan angle is maintained at the commanded value. Subsequent ABSOLUTE
pan commands are then relative to that rebased zero, not to an external
frame. Tilt and Roll are true-absolute commands (bounded by configured
limits).

Practical consequence:
- **Relative pan nudges are trivial**: command the desired delta directly.
- **External-frame absolute pan** (e.g. heading from GPS) requires correcting
  for the rebase offset. Use the orientation reported in `FreeflyAPI.status`
  (gimbal quaternion) to determine the true current angle and compute the
  offset before issuing an absolute command.

### 500 ms inactivity timeout → axes revert to DEFER

If no control packet is received for approximately 500 ms, all axes revert to
DEFER (single-operator Majestic). ABSOLUTE mode is explicitly **less tolerant
of input-timing variation than RATE** — the spec recommends fast, evenly
spaced inputs (~100 Hz).

**Important corollary — do not de-duplicate ABSOLUTE commands.**
A "send only on change" or de-dup optimization is unsafe for ABSOLUTE mode:
it starves the timing-sensitive stream and risks triggering the timeout. It
is acceptable for RATE only, and even then a sub-500 ms heartbeat must
re-send the last command so control is never dropped.

## Migrating from v1

### Breaking: `enable` is now enforced as a send-gate

In v1, `hamfly_control_t.enable` was documented as an explicit opt-in but was
never actually read — a caller who built a control struct and left `enable = 0`
still commanded the gimbal. As of v2 the field is honoured:

```c
hamfly_send_control(g, ctl);   /* returns HAMFLY_ERR_BAD_STATE if !ctl->enable */
```

**Action required:** set `ctl.enable = 1` once your app is ready to command
motion. `hamfly_control_init()` still zeroes it, so init-then-send now returns
`HAMFLY_ERR_BAD_STATE` instead of moving the gimbal.

`hamfly_kill()` deliberately bypasses this gate — an emergency stop is never
blocked by the opt-in.

### Non-breaking hardening in the same release

- **PSoC5 HAL**: `hamfly_psoc5_hal()` now zero-initialises the HAL struct.
  Previously `get_tick_ms` was left indeterminate and was called on the first
  valid RX frame (undefined behaviour — the `if (ptr)` guard is defeated by a
  garbage non-NULL pointer). Custom HAL constructors must zero any field they
  do not set.
- **Control values are clamped to ±1.0** before serialisation. Previously
  `|value| > 1.0` overflowed the int16 and wrapped, so `1.5` became `-16386` —
  a near-full-scale command in the *opposite* direction. It now saturates.
- **QX277 reserved bytes are zeroed** instead of being sent as stack garbage.
- **Varint decode is bounded** by the received body length.
- **TX build failures are surfaced** rather than discarded.
- `hamfly_write_attr_u8()` rejects `attr_id > 0x3FFF` (the two-byte varint path
  cannot encode it) with `HAMFLY_ERR_ENCODE`.

## Contributing

This API is designed for active development. Feel free to extend and modify as needed for your use case.

## Movi Community

With the apparent dissapearance of the forum, I thought it would be nice to accumulate other peoples efforts to work with the Movi Pro (and maybe someday other Freefly products).\

[Movi Pro ardunio custom remote](https://www.thingiverse.com/thing:4287654) by Caz on Thingverse.\
[Mini Movi Controller V2](https://www.thingiverse.com/thing:4672980) by Caz on Thingverse.\
Both seem to send commands over serial to a Mimic which communicates with the GCU. Projects under a Creative Commons license.\

Thingverse and other maker libraries have many custom components, brackets, and mounts to explore.
