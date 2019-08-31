# tactile_sensor_ros

ROS packages for the sun/unicampania force/tactile sensors.

This is a simplified version with no dependencies that just retrieves the voltages from the fingers. 

## Installation

Clone with `--recurse-submodules` option to download the serial submodule.

## Setup

To use the wired finger at full speed (500Hz) you need to install `setserial`:

```bash
sudo apt-get install setserial
```

When the finger is connected to the PC, take note of the dev file that Linux assigns to the device. It is something like `/dev/ttyUSB#` with `#` a number. You can check it using the command:
```bash
ls /dev/ttyUSB*
```
Typically the first finger that you connect takes the id `/dev/ttyUSB0` and the second takes id `/dev/ttyUSB1`.

You need the access rights to the device. `/dev/ttyUSB#` is owned by `root` and the group is `dialout`.
You can simply add your user to the `dialout` group:
```bash
sudo usermod -a -G groupName userName
```

## Run the ROS driver

You can run the driver using the launch file:
```bash
roslaunch sun_tactile_driver tactile_serial.launch
```

It has the following parameters:

| Parameter  | Description | Default |
| ------------- | ------------- | --------- |
| `serial_port`  | Device file associated with the finger  | `/dev/ttyUSB0` |
| `baud_rate`  | Baud rate for serial communication. **For the wired fingers it has to be 1000000**.  | `1000000` |
| `voltage_raw_topic` | Output topic for the raw voltages | `tactile_voltage/raw` |
| `voltage_rect_topic` | Output topic for the de-biased voltages | `tactile_voltage/rect"` |
| `action_compute_bias` | Action name that computes the bias* | `tactile_voltage/action_compute_bias` |

*the bias si computed at startup, you can force a new computation calling this action.

## License

This project is licensed under the GPLv3 License - see the [LICENSE.md](LICENSE.md) file for details
