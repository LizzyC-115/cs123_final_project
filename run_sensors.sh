#!/bin/bash
# Launch Pupper Realtime Voice System

echo "🚀 Starting Pupper Realtime System"
echo ""

# Trap Ctrl+C to kill all background processes
trap 'kill $(jobs -p) 2>/dev/null; exit' INT TERM

echo "Starting processes..."
echo ""

# Launch ROS2
echo "1. Launching ROS2..."
ros2 launch launch/launch.py > /tmp/ros2_launch.log 2>&1 &
ROS2_PID=$!

# Wait a moment for ROS2 to initialize
sleep 2

# Launch read_data.py
echo "2. Launching read_data.py to gather sensor data..."
python3 read_data.py > /tmp/read_data.log 2>&1 &
READ_PID=$!

# Launch read_data.py
echo "3. Launching movement_subscriber.py to read sensor data..."
python3 movement_subscriber.py > /tmp/movement_subscriber.log 2>&1 &
MOVEMENT_PID=$!

echo ""
echo "✅ All processes started!"
echo ""
echo "Logs:"
echo "  - ROS2: /tmp/ros2_launch.log"
echo "  - Voice: /tmp/realtime_voice.log"
echo "  - Karel: /tmp/karel_commander.log"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Wait for all background processes
wait

