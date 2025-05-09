#!/bin/bash

# Script to run unreachable cell validation tests

# Build the validation executable
echo "Building validation executable..."
cd "$(dirname "$0")"
mkdir -p build
cd build
cmake .. && make validate_unreachables

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# Run validation tests with different parameters
echo ""
echo "Running validation tests..."

# Small grid (10x10) test
echo ""
echo "==============================================="
echo "Test 1: 10x10 grid, 100 turns, verbose output"
echo "==============================================="
./validate_unreachables --width 10 --height 10 --turns 100 --verbose

# Medium grid (15x15) test
echo ""
echo "===============================================" 
echo "Test 2: 15x15 grid, 200 turns"
echo "==============================================="
./validate_unreachables --width 15 --height 15 --turns 200

# Large grid (20x20) test
echo ""
echo "==============================================="
echo "Test 3: 20x20 grid, 300 turns"
echo "==============================================="
./validate_unreachables --width 20 --height 20 --turns 300

echo ""
echo "All tests completed. Check the CSV files for detailed results."
