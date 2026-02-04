#!/bin/bash

# ---------------------------------------------------------
# RUN SIMULATED DEMO (GAZEBO + CUSTOM NAVIGATOR)
# ---------------------------------------------------------

set -e

echo "Launching simulated demo..."

# Resolve script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

SIM_SCRIPT="$PROJECT_ROOT/scripts/object_detection/test_simulate_custom_navigator.sh"

# Save starting directory for later return
START_DIR="$(pwd)"

echo "[INFO] Checking simulation script..."
if [ ! -f "$SIM_SCRIPT" ]; then
    echo "[ERROR] Simulation script not found:"
    echo "        $SIM_SCRIPT"
    exit 1
fi

echo "[INFO] Updating permissions..."
chmod +x "$SIM_SCRIPT"

echo "[INFO] Sourcing ROS Jazzy..."
source /opt/ros/jazzy/setup.bash

# Optional: source workspace if it exists
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    echo "[INFO] Sourcing workspace overlay..."
    source "$PROJECT_ROOT/install/setup.bash"
fi

echo "[INFO] Launching simulation pipeline..."
cd "$(dirname "$SIM_SCRIPT")" || {
    echo "[ERROR] Unable to enter simulation script directory."
    exit 1
}

./$(basename "$SIM_SCRIPT")

# Return to previous directory
cd "$START_DIR" || {
    echo "[WARNING] Could not return to starting directory: $START_DIR"
}

echo "[INFO] Simulated demo launch complete."
