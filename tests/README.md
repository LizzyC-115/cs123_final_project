# Test Scripts

Run these tests in order to verify the system after fixing the hardware.

## Prerequisites
1. SSH into the Pupper: `ssh pi@pupper3.local`
2. Start ROS2: `ros2 launch launch/launch.py`
3. In a new terminal, navigate to tests: `cd ~/cs123_final_project/tests`

---

## Test 1: Basic Movements
**Purpose:** Verify each KarelPupper movement works individually.

```bash
python3 test_basic_movements.py
```

Press Enter to test each movement one at a time.

---

## Test 2: Sensor Commander (Test Mode)
**Purpose:** Test the combined read_data.py without needing serial connection.

```bash
python3 test_sensor_commander.py
```

Robot will alternate turn_left/turn_right every 10 seconds.

---

## Test 3: Serial Simulation
**Purpose:** Type commands manually to simulate Pico W input.

```bash
python3 test_serial_simulation.py
```

Type commands like: `left`, `right`, `[TURN_LEFT]`, `forward`, `l`, `r`, `f`, `b`, `w`

---

## Test 4: Direct cmd_vel
**Purpose:** Bypass KarelPupper and publish directly to cmd_vel.

```bash
python3 test_cmd_vel_direct.py
```

If this works but Test 1 doesn't, the issue is in `karel.py`.

---

## Test 5: Full System (with Serial)
**Purpose:** Run the complete system with Pico W connected.

```bash
python3 test_full_system.py
```

Requires Pico W connected to `/dev/ttyACM0`.

---

## Quick Troubleshooting

| Symptom | Likely Cause |
|---------|--------------|
| No tests work | ROS2 not running or hardware issue |
| Test 4 works, Test 1 doesn't | Issue in `karel.py` |
| Test 1 works, Test 2 doesn't | Issue in `read_data.py` |
| All tests work, serial doesn't | Pico W connection issue |
