# PiezoMotorPMDCtrl

The `PiezoMotorPMDCtrl` is a Python-based device server for controlling a Piezo Motor using the TANGO Controls framework. It facilitates communication with the motor through a serial-to-Ethernet device, allowing for remote operation and monitoring of the motor's position, velocity, and status.

## Features

- **Device Communication**: Utilizes TCP/IP sockets for communication with a serial-to-Ethernet device, enabling remote control of the Piezo Motor.
- **Threaded Operation**: Implements separate threads for reading from and writing to the socket, ensuring responsive device interactions.
- **Attribute Monitoring**: Supports continuous monitoring and updating of device attributes such as position, encoder position, update rate, velocity, and control status.
- **Command Execution**: Provides TANGO commands for starting and stopping the motor, as well as sending custom requests to the device.

## Dependencies

- `socket`
- `queue`
- `time`
- `threading`
- `tango`

## Device Properties

- `moxa_host`: IP address of the Moxa IP/serial hub.
- `moxa_port`: Port of the Moxa IP/serial hub.
- `moxa_reconnect_delay`: Timeout before attempting to reconnect to the device.

## Device Attributes

- `position`: Motor position in microns.
- `enc_pos`: Encoder position in counts.
- `update_rate`: Rate at which device attributes are updated, in milliseconds.
- `velocity`: Motor velocity in microns per second.
- `status_ctrl`: Current status of the control system.

## Commands

- `Start`: Starts the motor.
- `Stop`: Stops the motor.
- `SendRequest`: Sends a custom request to the device and returns the response.

## Installation

Ensure you have a Python environment with the TANGO Controls framework installed. Clone this repository and run the server with:

```bash
python PiezoMotorPMDCtrl.py <Instance>