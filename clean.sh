#!/bin/bash
# Clean up __pycache__ and logs from the Modularized working code folder

echo "Cleaning up __pycache__ and logs..."

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORK_DIR="$SCRIPT_DIR/Modularized working code"

if [ -d "$WORK_DIR" ]; then
    cd "$WORK_DIR"
    
    # 1. Remove __pycache__ directories
    find . -name "__pycache__" -type d -exec rm -rf {} +
    echo "✓ Removed __pycache__ folders"

    # 2. Remove .pyc files
    find . -name "*.pyc" -type f -delete
    echo "✓ Removed .pyc files"

    echo "Cleanup complete!"
else
    echo "Error: Directory '$WORK_DIR' not found!"
    exit 1
fi
