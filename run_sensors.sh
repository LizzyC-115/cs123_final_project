#!/bin/bash
# Launch Pupper Sensor Commander System
# Combined serial reader + robot controller (single ROS2 node)

echo "🚀 Starting Pupper Sensor Commander"
echo ""

# Trap Ctrl+C to kill all background processes
trap 'kill $(jobs -p) 2>/dev/null; exit' INT TERM

# Check for --test flag
TEST_FLAG=""
if [ "$1" == "--test" ]; then
    TEST_FLAG="--test"
    echo "🧪 Running in TEST MODE (no serial required)"
fi

echo "Starting processes..."
echo ""

# Launch ROS2 control system
echo "1. Launching ROS2 control system..."
ros2 launch launch/launch.py > /tmp/ros2_launch.log 2>&1 &
ROS2_PID=$!

# Wait for ROS2 to initialize
sleep 3

# Launch combined sensor commander (reads serial + controls robot)
echo "2. Launching Sensor Commander (serial reader + robot control)..."
python3 read_data.py $TEST_FLAG 2>&1 | tee /tmp/sensor_commander.log &
SENSOR_PID=$!

echo ""
echo "✅ All processes started!"
echo ""
echo "Logs:"
echo "  - ROS2: /tmp/ros2_launch.log"
echo "  - Sensor Commander: /tmp/sensor_commander.log (also shown above)"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Wait for all background processes
wait

