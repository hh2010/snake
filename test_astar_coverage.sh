#!/bin/bash

# Script to test A* distance coverage compared to BFS

# Build the test executable
echo "Building A* coverage test..."
cd "$(dirname "$0")"
mkdir -p build
cd build
cmake .. && make astar_coverage_test

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# Run tests with different grid sizes
echo -e "\n=== Running A* Coverage Tests ==="

# Small grid (10x10)
echo -e "\n== Test with 10x10 grid =="
./astar_coverage_test --width 10 --height 10

# Medium grid (20x20)
echo -e "\n== Test with 20x20 grid =="
./astar_coverage_test --width 20 --height 20

# Large grid (30x30)
echo -e "\n== Test with 30x30 grid =="
./astar_coverage_test --width 30 --height 30

echo -e "\nAll tests completed."
