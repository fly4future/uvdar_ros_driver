# 📦 `uvdar_ros_driver`

The `uvdar_ros_driver` is a minimal ROS nodelet-based interface for reading UVDAR module messages over a USB connection. It matches the basic functionality of the UVDAR system running [UVDAR firmware](https://github.com/fly4future/uvdar_firmware), enabling integration with ROS ecosystems.

## Installation
Clone this repository and call:
```
gitman install
```

## Functionality

- **Nodelet-based**
  Designed as minimalistic C++ ROS nodelet.

- **UVDAR module handling**
  Designed for low-level handling of UVDAR module running UVDAR firmware.

- **USB Device Enumeration and connection**  
  Scans available USB devices and identifies UVDAR-compatible modules by serial number. It simply finds any USB device that have *ttyACM*. Connects specifically to the correct tty port of USB device defined by the user via `uvdar_usb_serial` parameter.

- **Diagnostic Mode**  
  When no serial is provided, lists available devices and exits — handy for initial setup and validation.

- **Message Decoding via LLCP**  
  Utilizes LLCP parsing logic to decode messages received over USB.

- **ROS Topic Publishing**  
  Decoded ranging messages are published to:
  ```
  /$UAV_NAME/uvdar_driver/distance
  ```
  using [UwbRangeStamped](https://github.com/fly4future/uvdar_ros_driver/blob/master/msg/UwbRangeStamped.msg) and [UwbRange](https://github.com/fly4future/uvdar_ros_driver/blob/master/msg/UwbRange.msg) messages.

## What’s Missing / To-Do

The current implementation is focused on data acquisition and publishing. 

Configuration and control are not yet implemented:

- [ ] **UVDAR Module Configuration Interface**
  - Set UVDAR device address
  - Configure UV LED blinking patterns
  - etc.

- [ ] **LLCP Encapsulation for Downlink**
  - Send structured commands from ROS to UVDAR module
  - Enable bidirectional control and diagnostics via LLCP and USB port