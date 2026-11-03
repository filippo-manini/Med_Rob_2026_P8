#!/bin/bash

# ============================================================
# run_controller.sh - Standalone launcher for ROS + Arduino
# ============================================================
# This script:
# 1. Detects the root of the repository.
# 2. Compiles and uploads the Arduino sketch.
# 3. Starts ROS (roscore) and waits until it's ready.
# 4. Launches the Aurora software.
# 5. Starts the rosserial node for Arduino communication.
# 6. Waits for /aurora_data topic to publish data.
# 7. Launches the main controller Python node.
# 8. Launches RViz with a pre-configured config file.
# ============================================================

# ------------------------------------------------------------
# 1. Detect repo root
# ------------------------------------------------------------
# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The repo root is the parent folder of scripts/
REPO_ROOT="$(realpath "$SCRIPT_DIR/..")"
echo "Repo root detected at: $REPO_ROOT"

# ------------------------------------------------------------
# 2. Set paths relative to repo root
# ------------------------------------------------------------
# Aurora software folder
AURORA_PATH="$REPO_ROOT/ndi_aurora_2_6_x"

# RViz config file
RVIZ_CONFIG="$REPO_ROOT/rviz/controller_visualizator.rviz"

# Arduino sketch folder
ARDUINO_PATH="$REPO_ROOT/arduino_code/arduino_communication"

# Controller Python code folder
CONTROLLER_PATH="$REPO_ROOT"

# ------------------------------------------------------------
# 3. Detect Arduino device
# ------------------------------------------------------------
DEVTTY=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)
if [[ -z "$DEVTTY" ]]; then
    echo "Error: no Arduino detected on ttyACM*"
    exit 1
else
    echo "Arduino detected on $DEVTTY"
fi

# ------------------------------------------------------------
# 4. Compile and upload Arduino sketch
# ------------------------------------------------------------
echo "Compiling Arduino code..."
arduino-cli compile --fqbn arduino:avr:uno "$ARDUINO_PATH/arduino_communication.ino" || { echo "Compile failed"; exit 1; }

echo "Uploading Arduino code..."
arduino-cli upload -p "$DEVTTY" --fqbn arduino:avr:uno "$ARDUINO_PATH/arduino_communication.ino" || { echo "Upload failed"; exit 1; }

echo "Arduino upload completed."

# ------------------------------------------------------------
# 5. Start roscore
# ------------------------------------------------------------
# Open a new terminal for roscore
gnome-terminal -- bash -c "echo 'Starting roscore...'; roscore; exec bash"

# Wait for roscore to start
until rostopic list > /dev/null 2>&1; do
    echo "Waiting for roscore..."
    sleep 1
done

# ------------------------------------------------------------
# 6. Launch Aurora software
# ------------------------------------------------------------
if [[ ! -x "$AURORA_PATH/run.sh" ]]; then
    echo "Error: $AURORA_PATH/run.sh not found or not executable!"
    exit 1
fi
gnome-terminal -- bash -c "cd '$AURORA_PATH' && ./run.sh; exec bash"

# ------------------------------------------------------------
# 7. Start rosserial for Arduino communication
# ------------------------------------------------------------
gnome-terminal -- bash -c "rosrun rosserial_python serial_node.py _port:=$DEVTTY _baud:=115200; exec bash"

# ------------------------------------------------------------
# 8. Wait until /aurora_data topic publishes data
# ------------------------------------------------------------
echo "Waiting for /aurora_data topic..."
until rostopic echo -n1 /aurora_data 2>/dev/null | grep . >/dev/null; do
    sleep 0.5
done
echo "/aurora_data is now publishing data!"

# Optionally, echo /aurora_data in a new terminal
gnome-terminal -- bash -c "rostopic echo /aurora_data; exec bash"

# ------------------------------------------------------------
# 9. Launch controller Python node
# ------------------------------------------------------------
gnome-terminal -- bash -c "cd '$CONTROLLER_PATH' && source ../../devel/setup.bash && rosrun filippo_controller main.py; exec bash"

# ------------------------------------------------------------
# 10. Launch RViz
# ------------------------------------------------------------
if [[ -f "$RVIZ_CONFIG" ]]; then
    gnome-terminal -- bash -c "rviz -d '$RVIZ_CONFIG'; exec bash"
else
    echo "Warning: RViz config not found. Launching default RViz."
    gnome-terminal -- bash -c "rviz; exec bash"
fi

echo "All processes started successfully."

