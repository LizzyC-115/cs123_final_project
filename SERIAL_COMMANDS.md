# Serial Command Control for Pupper

This document explains how to control Pupper using serial commands from a Raspberry Pi Pico W instead of voice commands.

## Overview

The serial command system replaces voice input with commands sent via serial from an external controller (Raspberry Pi Pico W). This allows you to control Pupper programmatically or with custom hardware interfaces.

## Architecture

```
Raspberry Pi Pico W  →  Serial (/dev/ttyACM0)  →  Serial Commander Node  →  gpt4_response_topic  →  Karel Commander  →  Pupper Robot
```

### Components

1. **Raspberry Pi Pico W**: Sends command strings via serial at 115200 baud
2. **Serial Commander Node** (`pupper_llm/serial_commander.py`): Reads serial data and publishes to ROS2 topic
3. **Karel Realtime Commander** (`pupper_llm/karel/karel_realtime_commander.py`): Executes commands on Pupper

## Supported Commands

The serial commander accepts the same command format as the voice system. Send one command per line:

### Movement Commands
- `move_forward` - Move forward one step
- `move_backward` - Move backward one step
- `turn_left` - Turn 90° left
- `turn_right` - Turn 90° right
- `move_left` - Strafe left
- `move_right` - Strafe right
- `stop` - Stop all movement and tracking

### Fun Commands
- `wiggle` - Wiggle tail
- `bob` - Bob up and down
- `bark` - Bark sound
- `dance` - Perform dance routine

### Posture Commands
- `sit` - Sit down
- `stand` - Stand up

### Tracking Commands (Lab 7)
- `start_tracking_person` - Track a person
- `start_tracking_dog` - Track a dog
- `start_tracking_<object>` - Track any COCO object (person, cat, bottle, etc.)
- `stop` - Stop tracking (same as stop movement)

### Vision Commands
- `describe_scene` - Describe what the camera sees

## Usage

### Method 1: Using the Launch File (Recommended)

Run the serial control system with the launch file:

```bash
ros2 launch serial_control.launch.py
```

This automatically starts:
- Serial Commander (reads from Pico W)
- Karel Commander (controls Pupper)

### Method 2: Running Nodes Individually

Terminal 1 - Start Serial Commander:
```bash
cd /home/user/cs123_final_project
python3 pupper_llm/serial_commander.py
```

Terminal 2 - Start Karel Commander:
```bash
cd /home/user/cs123_final_project
python3 pupper_llm/karel/karel_realtime_commander.py
```

### Method 3: Standalone Serial Reader (Testing)

For testing serial communication without ROS:
```bash
python3 read_data.py
```

## Pico W Setup

### Hardware Connection
1. Connect Raspberry Pi Pico W to Raspberry Pi 4 via USB
2. The Pico W should appear as `/dev/ttyACM0` (check with `ls /dev/ttyACM*`)

### Pico W Code Example (MicroPython)

```python
import time

# Send commands via USB serial
while True:
    # Example: send movement commands
    print("move_forward")
    time.sleep(2)

    print("turn_right")
    time.sleep(2)

    print("start_tracking_person")
    time.sleep(5)

    print("stop")
    time.sleep(2)
```

### Serial Format
- **Baud Rate**: 115200
- **Format**: One command per line, terminated with `\n`
- **Encoding**: UTF-8

## Troubleshooting

### Serial Port Not Found
```bash
# List available serial ports
ls /dev/ttyACM*

# If your Pico W is on a different port, edit serial_control.launch.py
# and change the serial_port parameter
```

### Permission Denied
```bash
# Add your user to the dialout group
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### No Commands Being Executed
1. Check that both nodes are running:
   ```bash
   ros2 node list
   ```
   Should show: `/serial_commander` and `/karel_realtime_commander`

2. Check that messages are being published:
   ```bash
   ros2 topic echo /gpt4_response_topic
   ```

3. Check serial commander logs for connection errors

### Commands Not Recognized
- Verify your command strings match the exact format (see "Supported Commands" above)
- Check the karel_realtime_commander logs for parsing errors
- Commands are case-sensitive

## Differences from Voice Control

| Feature | Voice Control | Serial Control |
|---------|--------------|----------------|
| Input Source | Microphone + OpenAI | Raspberry Pi Pico W |
| Latency | ~1-2 seconds | <100ms |
| Natural Language | Yes (GPT processes) | No (exact commands) |
| Internet Required | Yes | No |
| Cost | OpenAI API costs | Free |
| Flexibility | Very flexible | Precise control |

## Example Sequences

### Simple Navigation
```
move_forward
move_forward
turn_right
move_forward
stop
```

### Track and Follow
```
start_tracking_person
(Pupper will now autonomously follow the person)
stop
```

### Fun Routine
```
wiggle
bark
dance
bob
```

## Integration with Existing Code

The serial commander publishes to the same `gpt4_response_topic` that the voice system uses, so:
- No changes needed to `karel_realtime_commander.py`
- All existing command parsing and execution logic remains unchanged
- You can even run both systems simultaneously (though not recommended)

## Next Steps

1. Program your Pico W to send commands based on:
   - Button presses
   - Sensor readings
   - Wireless remote control
   - Scheduled routines
   - External API triggers

2. Extend the command set by modifying:
   - `karel_realtime_commander.py::extract_commands_from_line()` for parsing
   - `karel_realtime_commander.py::execute_command()` for execution
