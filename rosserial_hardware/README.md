# rosserial_hardware

**Usage example** of the embedded library (`rosserial_stm32/ros_lib`) inside example code:
an Arduino Uno firmware that talks rosserial, and a `ros2_control` `SystemInterface`
that connects to it so the board's LED can be driven from a standard GPIO controller.
It is a demonstration of how the pieces fit together, not a general-purpose hardware driver.

The `SystemInterface` talks the rosserial protocol directly over the serial port. The board's
data goes straight into the state/command interfaces: **no ROS topics are published or
subscribed** for the hardware link.

It reuses the same wire protocol as `rosserial_server`, but is self-contained and does not
modify it.

What is in here:

| Path | What it is |
|---|---|
| `example/arduino/main.cpp` | Arduino Uno firmware using the embedded `ros_lib` (`std_msgs/Bool`) |
| `src/`, `include/` | The `RosserialSystem` plugin (`ros2_control`) |
| `example/led/` | URDF and controller configuration for the LED |
| `launch/led.launch.py` | Starts `ros2_control` + `gpio_controller` |
| `example/fake_board.py` | Fake board on a pty, to test without hardware |

## How it works

```
Arduino  <-- serial, rosserial -->  RosserialSystem  <-->  controller_manager / controllers
 led_state (Bool, board publishes)   -> read()  -> state interface
 led       (Bool, board subscribes)  <- write() <- command interface
```

The plugin only translates values for a single digital output (an LED, a relay...):

* command: the `double` of the command interface becomes a `std_msgs/Bool` on `led` (> 0.5 is `true`).
* state: the `std_msgs/Bool` the board publishes on `led_state` becomes `1.0` / `0.0` in the state interface.

Declare exactly one command interface and at most one state interface (joint or gpio).

URDF parameters (`<hardware>`): `port` (`/dev/ttyACM0`), `baud` (`57600`, the default of the Arduino `NodeHandle`), `state_topic` (`led_state`), `command_topic` (`led`).

If the board disconnects, `read()`/`write()` keep returning OK and the link reconnects by itself.

## Example: Arduino Uno LED (GPIO controller)

1. Flash `example/arduino/main.cpp`. Put it in a PlatformIO project (`board = uno`,
   `framework = arduino`, see `example/arduino/platformio.ini`) that has the embedded
   library (`rosserial_stm32/ros_lib`) in its `lib/` folder, then `pio run -t upload`.
   It drives `LED_BUILTIN` (pin 13).
2. Build and launch (close anything else using the port, e.g. the serial monitor):
   ```
   colcon build --packages-select rosserial_msgs rosserial_hardware
   source install/setup.bash
   ros2 launch rosserial_hardware led.launch.py
   ```
   Edit `example/led/led.urdf` if the port is not `/dev/ttyACM0`. The Uno resets when the port opens; the plugin waits 2 s for it.
3. Turn the LED on / off:
   ```
   ros2 topic pub --once /gpio_controller/commands control_msgs/msg/DynamicInterfaceGroupValues \
     "{interface_groups: [led], interface_values: [{interface_names: [digital_output], values: [1.0]}]}"
   ```
   Feedback is on `/gpio_controller/gpio_states`.

Without hardware: `example/fake_board.py` emulates the board on a pty (prints the path; use it as `port`).

## Alternative: `rosserial_server` (topic mode)

The same firmware also works with the existing `serial_node`, which exposes the board's topics to ROS:

```
ros2 run rosserial_server serial_node --ros-args -p port:=/dev/ttyACM0 -p baud:=57600
ros2 topic pub --once /led std_msgs/msg/Bool "{data: true}"
ros2 topic echo /led_state
```

Only one of the two can own the serial port at a time.

## Limitations

* Example-level: a single `Bool` command and state; it is not a general joint/array transport.
* No services, parameters or multiple boards in one node (use one `<ros2_control>` block per board).
* Linux only (termios).
