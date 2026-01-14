#!/bin/bash
# Run the Universal Robot Visualizer & Control System directly from root
# Uses -B to prevent __pycache__ creation

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the working directory
WORK_DIR="$SCRIPT_DIR/Modularized working code"

# Navigate to the working directory
if [ -d "$WORK_DIR" ]; then
    cd "$WORK_DIR"
    echo "Running system from: $WORK_DIR"
    # Run main.py with -B flag (No bytecode)
    python3 -B main.py
else
    echo "Error: Directory '$WORK_DIR' not found!"
    exit 1
fi
